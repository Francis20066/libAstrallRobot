#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "astrall/backend/sim_backend.hpp"
#include "astrall/control/twist_command_mapper.hpp"

namespace {

using namespace std::chrono_literals;

astrall::Twist2D fromRosTwist(const geometry_msgs::msg::Twist& msg) {
    return astrall::Twist2D{msg.linear.x, msg.linear.y, msg.angular.z};
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

bool isStopped(const astrall::Twist2D& cmd) {
    constexpr double epsilon = 1.0e-9;
    return std::abs(cmd.vx) < epsilon && std::abs(cmd.vy) < epsilon && std::abs(cmd.w) < epsilon;
}

struct SensorMount {
    std::string frame_id;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
};

struct RayHit {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float intensity = 0.0f;
};

}  // namespace

class AstrallSimNode final : public rclcpp::Node {
public:
    AstrallSimNode()
        : Node("astrall_sim_node") {
        loadParameters();
        backend_ = std::make_unique<astrall::SimBackend>(sim_dt_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        configureRosInterfaces();
    }

private:
    void loadParameters() {
        limits_.max_vx = declare_parameter("max_vx", 1.0);
        limits_.max_vy = declare_parameter("max_vy", 0.5);
        limits_.max_w = declare_parameter("max_w", 1.0);
        sim_dt_ = std::max(0.001, declare_parameter("sim_dt", 0.02));
        publish_period_ms_ = declare_parameter("publish_period_ms", 20);
        cmd_vel_timeout_ms_ = declare_parameter("cmd_vel_timeout_ms", 500);
        odom_frame_id_ = declare_parameter("odom_frame_id", std::string("odom"));
        map_frame_id_ = declare_parameter("map_frame_id", odom_frame_id_);
        base_frame_id_ = declare_parameter("base_frame_id", std::string("base_link"));
        imu_frame_id_ = declare_parameter("imu_frame_id", std::string("imu_link"));
        publish_pointcloud_ = declare_parameter("publish_pointcloud", true);
        publish_map_ = declare_parameter("publish_map", true);
        lidar_range_m_ = std::max(0.5, declare_parameter("lidar_range_m", 8.0));
        lidar_ray_count_ = static_cast<int>(
            std::max<int64_t>(16, declare_parameter("lidar_ray_count", 181)));
        lidar_fov_rad_ = std::max(0.1, declare_parameter("lidar_fov_rad", 3.839724354));
        map_resolution_m_ = std::max(0.02, declare_parameter("map_resolution_m", 0.1));
        map_width_m_ = std::max(2.0, declare_parameter("map_width_m", 16.0));
        map_height_m_ = std::max(2.0, declare_parameter("map_height_m", 12.0));
    }

    void configureRosInterfaces() {
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::QoS(10),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                onCmdVel(*msg);
            });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/astrall/imu", rclcpp::SensorDataQoS());
        wheel_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/astrall/wheel_speeds", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/astrall/status", 10);
        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        front_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/front/points_raw",
            rclcpp::SensorDataQoS());
        rear_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/rear/points_raw",
            rclcpp::SensorDataQoS());

        rclcpp::QoS map_qos(1);
        map_qos.transient_local().reliable();
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
        publishMap(now());

        last_cmd_time_ = now();
        sim_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(sim_dt_)),
            [this]() { simulationTick(); });
        publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(std::max(1, publish_period_ms_)),
            [this]() { publishTick(); });
    }

    void onCmdVel(const geometry_msgs::msg::Twist& msg) {
        active_cmd_ = astrall::clampTwist(fromRosTwist(msg), limits_);
        last_cmd_time_ = now();
        timed_out_stop_sent_ = false;
        status_text_ = isStopped(active_cmd_) ? "sim_idle" : "sim_active";
    }

    void simulationTick() {
        if (!backend_) {
            return;
        }

        const auto elapsed_ms = (now() - last_cmd_time_).nanoseconds() / 1000000;
        if (elapsed_ms > cmd_vel_timeout_ms_) {
            if (!timed_out_stop_sent_) {
                backend_->stop();
                active_cmd_ = astrall::Twist2D{};
                status_text_ = "cmd_vel_timeout_stop";
                timed_out_stop_sent_ = true;
            }
            return;
        }

        backend_->sendVelocity(active_cmd_);
    }

    void publishTick() {
        if (!backend_) {
            return;
        }

        const auto stamp = now();
        const astrall::Pose2D pose = backend_->getCurrentPose();
        const astrall::Twist2D cmd = backend_->lastCommand();

        publishOdom(stamp, pose, cmd);
        publishTf(stamp, pose);
        publishImu(stamp, pose, cmd);
        publishWheelSpeeds(stamp, cmd);
        publishPointClouds(stamp, pose);
        publishStatus();
        publishDiagnostics(stamp);
    }

    void publishOdom(const rclcpp::Time& stamp,
                     const astrall::Pose2D& pose,
                     const astrall::Twist2D& cmd) {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_id_;
        msg.child_frame_id = base_frame_id_;
        msg.pose.pose.position.x = pose.x;
        msg.pose.pose.position.y = pose.y;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation = yawToQuaternion(pose.theta);
        msg.twist.twist.linear.x = cmd.vx;
        msg.twist.twist.linear.y = cmd.vy;
        msg.twist.twist.angular.z = cmd.w;
        odom_pub_->publish(msg);
    }

    void publishTf(const rclcpp::Time& stamp, const astrall::Pose2D& pose) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = odom_frame_id_;
        transform.child_frame_id = base_frame_id_;
        transform.transform.translation.x = pose.x;
        transform.transform.translation.y = pose.y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = yawToQuaternion(pose.theta);
        tf_broadcaster_->sendTransform(transform);
    }

    void publishImu(const rclcpp::Time& stamp,
                    const astrall::Pose2D& pose,
                    const astrall::Twist2D& cmd) {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = imu_frame_id_;
        msg.orientation = yawToQuaternion(pose.theta);
        msg.angular_velocity.z = cmd.w;
        imu_pub_->publish(msg);
    }

    void publishWheelSpeeds(const rclcpp::Time&, const astrall::Twist2D& cmd) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {
            static_cast<float>(cmd.vx),
            static_cast<float>(cmd.vy),
            static_cast<float>(cmd.w),
            0.0f,
        };
        wheel_pub_->publish(msg);
    }

    void publishMap(const rclcpp::Time& stamp) {
        if (!publish_map_) {
            return;
        }

        const int width = static_cast<int>(std::ceil(map_width_m_ / map_resolution_m_));
        const int height = static_cast<int>(std::ceil(map_height_m_ / map_resolution_m_));

        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = map_frame_id_;
        msg.info.resolution = static_cast<float>(map_resolution_m_);
        msg.info.width = static_cast<std::uint32_t>(width);
        msg.info.height = static_cast<std::uint32_t>(height);
        msg.info.origin.position.x = -0.5 * map_width_m_;
        msg.info.origin.position.y = -0.5 * map_height_m_;
        msg.info.origin.orientation.w = 1.0;
        msg.data.resize(static_cast<std::size_t>(width * height), 0);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const double world_x = msg.info.origin.position.x + (x + 0.5) * map_resolution_m_;
                const double world_y = msg.info.origin.position.y + (y + 0.5) * map_resolution_m_;
                msg.data[static_cast<std::size_t>(y * width + x)] =
                    isOccupied(world_x, world_y) ? 100 : 0;
            }
        }

        map_pub_->publish(msg);
    }

    void publishPointClouds(const rclcpp::Time& stamp, const astrall::Pose2D& pose) {
        if (!publish_pointcloud_) {
            return;
        }

        publishCloudForSensor(stamp, pose, front_lidar_, front_cloud_pub_);
        publishCloudForSensor(stamp, pose, rear_lidar_, rear_cloud_pub_);
    }

    void publishCloudForSensor(
        const rclcpp::Time& stamp,
        const astrall::Pose2D& base_pose,
        const SensorMount& sensor,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) {
        const std::vector<RayHit> hits = raycastSensor(base_pose, sensor);

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stamp;
        cloud.header.frame_id = sensor.frame_id;
        cloud.height = 1;
        cloud.width = static_cast<std::uint32_t>(hits.size());
        cloud.is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(hits.size());

        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
        for (const RayHit& hit : hits) {
            *x = hit.x;
            *y = hit.y;
            *z = hit.z;
            *intensity = hit.intensity;
            ++x;
            ++y;
            ++z;
            ++intensity;
        }

        publisher->publish(cloud);
    }

    std::vector<RayHit> raycastSensor(const astrall::Pose2D& base_pose, const SensorMount& sensor) const {
        const double c = std::cos(base_pose.theta);
        const double s = std::sin(base_pose.theta);
        const double sensor_world_x = base_pose.x + c * sensor.x - s * sensor.y;
        const double sensor_world_y = base_pose.y + s * sensor.x + c * sensor.y;
        const double sensor_world_yaw = base_pose.theta + sensor.yaw;
        const double start_angle = -0.5 * lidar_fov_rad_;
        const double angle_step = lidar_ray_count_ > 1 ? lidar_fov_rad_ / (lidar_ray_count_ - 1) : 0.0;
        const double range_step = std::max(0.03, map_resolution_m_ * 0.5);

        std::vector<RayHit> hits;
        hits.reserve(static_cast<std::size_t>(lidar_ray_count_));

        for (int ray = 0; ray < lidar_ray_count_; ++ray) {
            const double local_angle = start_angle + ray * angle_step;
            const double world_angle = sensor_world_yaw + local_angle;
            const double ray_c = std::cos(world_angle);
            const double ray_s = std::sin(world_angle);

            for (double range = range_step; range <= lidar_range_m_; range += range_step) {
                const double world_x = sensor_world_x + range * ray_c;
                const double world_y = sensor_world_y + range * ray_s;
                if (!insideMap(world_x, world_y) || isOccupied(world_x, world_y)) {
                    hits.push_back(toSensorHit(
                        world_x,
                        world_y,
                        sensor_world_x,
                        sensor_world_y,
                        sensor_world_yaw,
                        sensor.z,
                        range));
                    break;
                }
            }
        }

        return hits;
    }

    RayHit toSensorHit(double world_x,
                       double world_y,
                       double sensor_world_x,
                       double sensor_world_y,
                       double sensor_world_yaw,
                       double sensor_z,
                       double range) const {
        const double dx = world_x - sensor_world_x;
        const double dy = world_y - sensor_world_y;
        const double c = std::cos(-sensor_world_yaw);
        const double s = std::sin(-sensor_world_yaw);
        return RayHit{
            static_cast<float>(c * dx - s * dy),
            static_cast<float>(s * dx + c * dy),
            static_cast<float>(-sensor_z),
            static_cast<float>(std::max(0.0, 255.0 * (1.0 - range / lidar_range_m_))),
        };
    }

    bool insideMap(double x, double y) const {
        return x >= -0.5 * map_width_m_ && x <= 0.5 * map_width_m_ &&
               y >= -0.5 * map_height_m_ && y <= 0.5 * map_height_m_;
    }

    bool isOccupied(double x, double y) const {
        if (!insideMap(x, y)) {
            return true;
        }

        const double half_width = 0.5 * map_width_m_;
        const double half_height = 0.5 * map_height_m_;
        if (std::abs(x) > half_width - 0.2 || std::abs(y) > half_height - 0.2) {
            return true;
        }

        return inBox(x, y, 2.6, 1.4, 0.5, 2.6) ||
               inBox(x, y, -2.2, -1.2, 1.2, 1.2) ||
               inBox(x, y, 0.0, 3.0, 4.4, 0.25) ||
               inBox(x, y, -4.4, 2.0, 0.5, 2.0) ||
               inBox(x, y, 4.4, -2.4, 0.7, 1.8) ||
               inCircle(x, y, 1.0, -3.0, 0.45);
    }

    bool inBox(double x, double y, double center_x, double center_y, double width, double height) const {
        return std::abs(x - center_x) <= 0.5 * width && std::abs(y - center_y) <= 0.5 * height;
    }

    bool inCircle(double x, double y, double center_x, double center_y, double radius) const {
        const double dx = x - center_x;
        const double dy = y - center_y;
        return dx * dx + dy * dy <= radius * radius;
    }

    void publishStatus() {
        std_msgs::msg::String msg;
        msg.data = status_text_;
        status_pub_->publish(msg);
    }

    void publishDiagnostics(const rclcpp::Time& stamp) {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = stamp;

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "astrall_sim";
        status.hardware_id = "sim";
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = status_text_;
        array.status.push_back(status);
        diagnostics_pub_->publish(array);
    }

    std::unique_ptr<astrall::SimBackend> backend_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    astrall::VelocityLimits limits_;
    astrall::Twist2D active_cmd_;

    double sim_dt_ = 0.02;
    int publish_period_ms_ = 20;
    int cmd_vel_timeout_ms_ = 500;
    bool timed_out_stop_sent_ = false;
    std::string odom_frame_id_ = "odom";
    std::string base_frame_id_ = "base_link";
    std::string imu_frame_id_ = "imu_link";
    std::string status_text_ = "sim_idle";
    rclcpp::Time last_cmd_time_;
    bool publish_pointcloud_ = true;
    bool publish_map_ = true;
    double lidar_range_m_ = 8.0;
    int lidar_ray_count_ = 181;
    double lidar_fov_rad_ = 3.839724354;
    double map_resolution_m_ = 0.1;
    double map_width_m_ = 16.0;
    double map_height_m_ = 12.0;
    std::string map_frame_id_ = "odom";
    SensorMount front_lidar_{"front_lidar_link", 0.444, 0.0, 0.288, 0.0};
    SensorMount rear_lidar_{"rear_lidar_link", -0.420, 0.0, 0.339, 3.141592653589793};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr front_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rear_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr sim_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AstrallSimNode>());
    rclcpp::shutdown();
    return 0;
}

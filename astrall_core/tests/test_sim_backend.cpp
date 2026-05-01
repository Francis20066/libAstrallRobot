#include <cassert>
#include <cmath>

#include "astrall/backend/sim_backend.hpp"

namespace {

bool near(double actual, double expected) {
    return std::abs(actual - expected) < 1.0e-9;
}

}  // namespace

int main() {
    astrall::SimBackend backend(0.1);

    backend.sendVelocity(astrall::Twist2D{1.0, -0.5, 0.25});
    backend.sendVelocity(astrall::Twist2D{1.0, -0.5, 0.25});

    astrall::Pose2D pose = backend.getCurrentPose();
    assert(near(pose.x, 0.2));
    assert(near(pose.y, -0.1));
    assert(near(pose.theta, 0.05));

    backend.stop();
    assert(near(backend.lastCommand().vx, 0.0));
    assert(near(backend.lastCommand().vy, 0.0));
    assert(near(backend.lastCommand().w, 0.0));

    pose = backend.getCurrentPose();
    assert(near(pose.x, 0.2));
    assert(near(pose.y, -0.1));
    assert(near(pose.theta, 0.05));

    return 0;
}

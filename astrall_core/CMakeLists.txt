cmake_minimum_required(VERSION 3.20)

project(astrall_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ASTRALL_BUILD_DEMOS "Build core demo executables" ON)
option(ASTRALL_BUILD_TESTS "Build core CTest tests" ON)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(ASTRALL_ENABLE_SDK_DEFAULT ON)
else()
    set(ASTRALL_ENABLE_SDK_DEFAULT OFF)
endif()

option(ASTRALL_ENABLE_SDK
    "Build Astrall SDK integration. Defaults ON on Linux and OFF elsewhere."
    ${ASTRALL_ENABLE_SDK_DEFAULT}
)

find_package(yaml-cpp REQUIRED)

if(ASTRALL_ENABLE_SDK)
    add_subdirectory(lib/astrall_sdk)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
endif()

if(TARGET yaml-cpp::yaml-cpp)
    set(ASTRALL_YAML_TARGET yaml-cpp::yaml-cpp)
else()
    set(ASTRALL_YAML_TARGET yaml-cpp)
endif()

set(ASTRALL_CORE_SOURCES
    src/backend/backend_factory.cpp
    src/backend/sim_backend.cpp
    src/backend/real_backend.cpp
    src/control/controller.cpp
    src/control/twist_command_mapper.cpp
    src/planning/straight_line_planner.cpp
    src/navigation/navigator.cpp
    src/behavior/state_machine.cpp
    src/device/dummy_camera.cpp
    src/device/dummy_radar.cpp
    src/runtime.cpp
)

set(ASTRALL_CORE_LIBS
    ${ASTRALL_YAML_TARGET}
)
set(ASTRALL_CORE_PRIVATE_LIBS)

if(ASTRALL_ENABLE_SDK)
    list(APPEND ASTRALL_CORE_SOURCES
        src/sdk/astrall_sdk_wrapper.cpp
        src/device/real_camera.cpp
    )
    list(APPEND ASTRALL_CORE_PRIVATE_LIBS
        astrall::sdk
        opencv_core
        opencv_imgproc
        opencv_videoio
    )
endif()

add_library(astrall_core
    ${ASTRALL_CORE_SOURCES}
)

set_target_properties(astrall_core
    PROPERTIES
        POSITION_INDEPENDENT_CODE ON
)

target_include_directories(astrall_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(astrall_core
    PRIVATE
        ${ASTRALL_CORE_LIBS}
        ${ASTRALL_CORE_PRIVATE_LIBS}
)

if(ASTRALL_ENABLE_SDK)
    target_compile_definitions(astrall_core PUBLIC ASTRALL_ENABLE_SDK=1)
else()
    target_compile_definitions(astrall_core PUBLIC ASTRALL_ENABLE_SDK=0)
endif()

if(ASTRALL_BUILD_DEMOS)
    add_executable(astrall_cpp_demo examples/cpp_demo.cpp)
    target_link_libraries(astrall_cpp_demo PRIVATE astrall_core)
endif()

if(ASTRALL_BUILD_TESTS)
    enable_testing()
    find_package(Threads REQUIRED)

    add_executable(astrall_twist_mapper_test tests/test_twist_command_mapper.cpp)
    target_link_libraries(astrall_twist_mapper_test PRIVATE astrall_core)
    add_test(NAME astrall_twist_mapper_test COMMAND astrall_twist_mapper_test)

    add_executable(astrall_navigator_status_test tests/test_navigator_status.cpp)
    target_link_libraries(astrall_navigator_status_test PRIVATE astrall_core)
    add_test(NAME astrall_navigator_status_test COMMAND astrall_navigator_status_test)

    add_executable(astrall_backend_interface_test tests/test_backend_interface.cpp)
    target_link_libraries(astrall_backend_interface_test PRIVATE astrall_core)
    add_test(NAME astrall_backend_interface_test COMMAND astrall_backend_interface_test)

    add_executable(astrall_sdk_wrapper_fake_test
        tests/test_astrall_sdk_wrapper_fake.cpp
        src/sdk/astrall_sdk_wrapper.cpp
    )
    target_compile_definitions(astrall_sdk_wrapper_fake_test PRIVATE ASTRALL_ENABLE_SDK=1)
    target_include_directories(astrall_sdk_wrapper_fake_test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/astrall_sdk/include
    )
    target_link_libraries(astrall_sdk_wrapper_fake_test PRIVATE Threads::Threads)
    add_test(NAME astrall_sdk_wrapper_fake_test COMMAND astrall_sdk_wrapper_fake_test)

    add_executable(astrall_real_backend_fake_sdk_test
        tests/test_real_backend_fake_sdk.cpp
        src/backend/real_backend.cpp
        src/sdk/astrall_sdk_wrapper.cpp
    )
    target_compile_definitions(astrall_real_backend_fake_sdk_test PRIVATE ASTRALL_ENABLE_SDK=1)
    target_include_directories(astrall_real_backend_fake_sdk_test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/astrall_sdk/include
    )
    target_link_libraries(astrall_real_backend_fake_sdk_test PRIVATE Threads::Threads)
    add_test(NAME astrall_real_backend_fake_sdk_test COMMAND astrall_real_backend_fake_sdk_test)
endif()

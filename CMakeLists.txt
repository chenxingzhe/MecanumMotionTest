cmake_minimum_required(VERSION 3.2)

include_directories(${Protobuf2Dir}/include
D:/opencv3/opencv/build/include
${NEAT_SOURCE_DIR}/external_libs/Eigen/include
)

link_directories(${Protobuf2Dir}/lib
D:/opencv3/opencv/build/x86/vc12/lib
)

file(GLOB_RECURSE motionSrc ./*.cpp)
file(GLOB_RECURSE motionInc ./*.h)
add_executable(MecanumMotionTest ${motionSrc} ${motionInc})

# the lib is dependent on MecanumMotion.
target_link_libraries(MecanumMotionTest MecanumMotion
    ConfigAndSetting
    os_message.lib os_node.lib suro_rpc.lib         # suroROS
    libprotobuf.lib                                 # protobuf
	opencv_world300.lib
	opencv_ts300.lib
	opencv_xphoto300.lib
    )

set_target_properties(MecanumMotionTest
    PROPERTIES
    DEBUG_POSTFIX "_D"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${NEAT_SOURCE_DIR}/bin/Debug"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${NEAT_SOURCE_DIR}/bin/Release"
)

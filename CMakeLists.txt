cmake_minimum_required(VERSION 2.8.3)
project(sweep_ros)

set(SWEEP_SDK_PATH "sweep-sdk-cpp")
set(CPP_SERIAL_PATH "sweep-sdk-cpp/src/serial")

FILE(GLOB SWEEP_SDK_SRC
        "${SWEEP_SDK_PATH}/src/*.cc"
        "${CPP_SERIAL_PATH}/*.cc"
        "${CPP_SERIAL_PATH}/arch/*.cc"
        )

find_package(catkin REQUIRED COMPONENTS
        rosconsole
        roscpp
        sensor_msgs
        pcl_msgs
        )

find_package(PCL REQUIRED)

include_directories(
        ${SWEEP_SDK_PATH}/inc
        ${SWEEP_SDK_PATH}/inc/serial
        ${SWEEP_SDK_PATH}/src
        ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
        ${PCL_LIBRARY_DIRS}
)

catkin_package()

add_executable(sweep_node src/node.cpp ${SWEEP_SDK_SRC})

target_link_libraries(sweep_node ${catkin_LIBRARIES})


install(TARGETS sweep_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY sweep-sdk-cpp
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS
        )

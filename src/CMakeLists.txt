cmake_minimum_required(VERSION 3.5)
project(frustum_projector CUDA CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -g -O3")

set(OpenCV_DIR "~/libraries/libopencv-4.4.0/lib/cmake/opencv4/")
find_package(OpenCV 4.4 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


# Thrust
set(THRUST_COMPONENTS CPP OMP TBB)
set(THRUST_OPTIONAL_COMPONENTS CUDA)
add_subdirectory(thrust)



find_package(catkin REQUIRED COMPONENTS
        rospy
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        tf
        tf_conversions
        tf2
        tf2_eigen
        cv_bridge
        image_transport
        )



catkin_package(
        CATKIN_DEPENDS
        rospy
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        tf
        tf_conversions
        tf2
        tf2_eigen
        cv_bridge
        image_transport
)

include_directories(
        /home/goktug/projects/jsk_recognition_msgs/install/include
        /home/goktug/projects/vision_msgs/install/include
        /home/goktug/projects/autoware_ws/install/autoware_msgs/include
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/FrustumProjector.cpp
        src/LidcamHelpers.cpp
        src/JskHelper.cpp
        src/PclStuff.cpp
        src/QueueCloud.cpp)

thrust_create_target(
        ThrustTBB
        HOST
        TBB
        DEVICE
        CUDA
        IGNORE_CUB_VERSION_CHECK
)        

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ThrustTBB
        )

thrust_update_system_found_flags()

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

cmake_minimum_required(VERSION 2.8)
project(wfi_from_depth_sensor)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED 
        cv_bridge
        dynamic_reconfigure
        image_geometry
        image_transport
        nodelet
        nodelet_topic_tools
        roscpp
        sensor_msgs
        message_generation)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

add_message_files(
	DIRECTORY
	msg
	FILES
	FourierCoefsMsg.msg
)

generate_messages(DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
        cfg/Scan.cfg)

catkin_package(
        CATKIN_DEPENDS
        image_geometry
        roscpp
        sensor_msgs
        message_runtime
        DEPENDS
        OpenCV
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Temporary fix for DataType deprecation in OpenCV 3.3.1. We continue to use the deprecated form for now because
# the new one is not available in OpenCV 2.4 (on Trusty).
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)

# Nodelet library
add_library(${PROJECT_NAME} src/libwfi_from_depth_sensor/processor.cpp
				src/nodelets/scan.cpp
				src/libwfi_from_depth_sensor/advertisement_checker.cpp	
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Standalone node
add_executable(wfi_from_depth_sensor_exe src/nodes/wfi_from_depth_sensor.cpp)
target_link_libraries(wfi_from_depth_sensor_exe ${PROJECT_NAME}  ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
SET_TARGET_PROPERTIES(wfi_from_depth_sensor_exe PROPERTIES OUTPUT_NAME wfi_from_depth_sensor)
install(TARGETS wfi_from_depth_sensor_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)


cmake_minimum_required(VERSION 2.8.3)
project(interact_slam)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  tf_conversions
  pcl_conversions
  message_generation
)

find_package(
    Qt5 REQUIRED COMPONENTS  Widgets
)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


add_service_files(FILES Order.srv)
generate_messages(DEPENDENCIES std_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES interact_slam
#  CATKIN_DEPENDS cv_bridge image_transport message_filters roscpp sensor_msgs std_msgs tf tf_conversions
#  DEPENDS system_lib
)




include_directories(
    include/interact_slam
    include
    ${catkin_INCLUDE_DIRS}
    /usr/local/include
     /usr/include/pcl-1.7
      /usr/include/eigen3
      /usr/include/vtk-6.2
    /usr/local/include/opencv
    /usr/local/include/opencv2
)


set(LIBS
        -lpcl_common
        -lpcl_search
        -lpcl_features
        -lpcl_segmentation
        -lpcl_recognition
        -lpcl_visualization
            -L/usr/lib/x86_64-linux-gnu/
            -L/usr/lib/x86_64-linux-gnu/
             -lboost_system
             -lboost_thread
         -lvtkRenderingCore-6.2
         -lvtkCommonDataModel-6.2
         -lvtkCommonMath-6.2
         -lvtkCommonCore-6.2
         -lvtkGUISupportQt-6.2
         /usr/local/lib/libopencv_highgui.so
         /usr/local/lib/libopencv_core.so
         /usr/local/lib/libopencv_imgproc.so
        /usr/local/lib/libopencv_imgcodecs.so
         Qt5::Widgets
)


set(SOURCES
  src/Console.cpp
  src/mainwindow.cpp
)

set(FORMS
  src/mainwindow.ui
)

add_library(roshandle src/roshandle.h src/roshandle.cpp)
target_link_libraries(roshandle ${catkin_LIBRARIES} ${LIBS})
add_dependencies(roshandle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set(LIBS3
        roshandle
)


add_executable(${PROJECT_NAME}_node ${SOURCES} ${FORMS})

target_link_libraries(
    ${PROJECT_NAME}_node
    ${catkin_LIBRARIES} ${LIBS}  ${LIBS3}
)


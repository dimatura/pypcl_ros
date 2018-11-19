cmake_minimum_required(VERSION 3.5.1)
project(pypcl_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  #eigen_catkin
  image_geometry
  pybind11_catkin
  pybind11
  pyrosmsg
  roscpp
  rospy
  sensor_msgs
  tf2
  visualization_msgs
)

find_package(NUMPY REQUIRED)
find_package(PCL REQUIRED)
#find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

catkin_package(
    #INCLUDE_DIRS include
    # TODO figure this one out
    #LIBRARIES libpypcl_ros
    CATKIN_DEPENDS cmake_modules pybind11_catkin roscpp rospy
    DEPENDS NUMPY PCL # Eigen3
    #CFG_EXTRAS config_extras.cmake
)


pybind_add_module(libpypcl_ros
    src_wrap/converters.cpp
    src_wrap/module.cpp
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

target_link_libraries(libpypcl_ros
    PRIVATE
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${EIGEN_INCLUDE_DIRS}
)

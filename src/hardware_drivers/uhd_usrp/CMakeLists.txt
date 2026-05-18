cmake_minimum_required(VERSION 3.8)
project(uhd_usrp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rf_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)
pkg_check_modules(UHD REQUIRED uhd)

add_executable(usrp_driver_node src/usrp_driver_node.cpp)

target_include_directories(usrp_driver_node PRIVATE ${UHD_INCLUDE_DIRS})
target_link_directories(usrp_driver_node PRIVATE ${UHD_LIBRARY_DIRS})
target_link_libraries(usrp_driver_node ${UHD_LIBRARIES} Boost::thread)
ament_target_dependencies(usrp_driver_node rclcpp rf_msgs)

install(TARGETS usrp_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(rf_pipeline)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(rf_pipeline)

install(PROGRAMS
  scripts/rf_detector
  scripts/scoreboard
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
  FILES_MATCHING PATTERN "*.pt" PATTERN "*.torchscript"
)

ament_package()

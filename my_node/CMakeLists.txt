cmake_minimum_required(VERSION 3.8)
project(my_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

#- For C++ -------------------------------------------------#
#include cpp in <include> directory
# include_directories(include)

#create cpp executables
# add_executable(cpp_exe src/cpp_node.cpp)
# ament_target_dependencies(cpp_exe rclcpp)

#install cpp executables
# install(TARGETS
#   cpp_exe
#   DESTINATION lib/${PROJECT_NAME}
# )

#- For Python ---------------------------------------------#
#install py modues
ament_python_install_package(${PROJECT_NAME})

#install py executables
install(PROGRAMS
  # Add your script below
  # scripts/turtle_move.py


  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY 
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()

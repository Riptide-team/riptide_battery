cmake_minimum_required(VERSION 3.8)
project(riptide_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
include(FetchContent REQUIRED) # once in the project to include the module

FetchContent_Declare(rtac_asio GIT_REPOSITORY https://github.com/pnarvor/rtac_asio.git GIT_TAG master)
FetchContent_MakeAvailable(rtac_asio)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

# Compile the driver
add_executable(${PROJECT_NAME} src/riptide_battery.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json rtac_asio)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()

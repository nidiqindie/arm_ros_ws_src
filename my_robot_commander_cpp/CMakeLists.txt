cmake_minimum_required(VERSION 3.8)
project(my_robot_commander_cpp)
# 确保在安装时保留 RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 将你的 Qt 库路径添加到运行时的搜索路径中
# 注意替换为你实际的 lib 文件夹路径
list(APPEND CMAKE_INSTALL_RPATH "/home/pope/Qt/6.11.0/gcc_64/lib")

set(Qt6_DIR "/home/pope/Qt/6.11.0/gcc_64/lib/cmake/Qt6")
list(APPEND CMAKE_PREFIX_PATH "/home/pope/Qt/6.11.0/gcc_64")



find_library(MY_CUSTOM_LIB NAMES small_scara_interface HINTS /usr/lib)
if(NOT MY_CUSTOM_LIB)
    message(FATAL_ERROR "自定义库 libexample.so 未在 /usr/lib 中找到！")
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core) # 如果需要界面，增加 Widgets
# 3. 开启 Qt 自动编译选项
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)

add_executable(test_moveit_oop src/command_temple.cpp)
ament_target_dependencies(test_moveit_oop rclcpp moveit_ros_planning_interface)

# 4. 添加可执行文件
add_executable(test_arm src/test_arm.cpp)
# 5. 包含头文件路径
target_include_directories(test_arm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# 6. 链接依赖库
# 注意：Qt6::Core 必须显式链接
target_link_libraries(test_arm
  Qt6::Core
  ${MY_CUSTOM_LIB}  # 链接刚才找到的库路径
)
install(TARGETS
  test_moveit_oop
  test_moveit
  test_arm
  DESTINATION lib/${PROJECT_NAME}/)
ament_package()

# 指定 CMake 的最低版本要求。ROS 2 Jazzy 建议至少 3.8
cmake_minimum_required(VERSION 3.8)

# 定义项目名称，通常与 package.xml 中的包名一致
project(my_robot_commander_cpp)
# --- 运行时路径 (RPATH) 配置 ---
# 当你运行程序时，系统默认去 /usr/lib 找库。
# 这一句告诉编译器：在安装程序时，把我们手动链接的库路径也记在二进制文件里。
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 将你的 Qt 6.11 库路径追加到 RPATH 中。
# 这样你运行 ros2 run 时，程序会自动去这个目录下找 libQt6Core.so.6，而不会误用系统自带的旧库。
list(APPEND CMAKE_INSTALL_RPATH "/home/pope/Qt/6.11.0/gcc_64/lib")

# --- 编译时路径配置 ---
# 告诉 CMake 到哪里去定位 Qt6 的配置文件（FindQt6.cmake 等）
set(Qt6_DIR "/home/pope/Qt/6.11.0/gcc_64/lib/cmake/Qt6")
# 告诉 CMake 在搜索依赖包时，优先去这个目录下寻找
list(APPEND CMAKE_PREFIX_PATH "/home/pope/Qt/6.11.0/gcc_64")


# 在 /usr/lib 下寻找名为 libsmall_scara_interface.so 的库
# 找到后，会将该库的完整路径存入变量 MY_CUSTOM_LIB 中
find_library(MY_CUSTOM_LIB NAMES small_scara_interface HINTS /usr/lib)
# 容错检查：如果没找到库，直接报错停止编译，省得后面链接时才报错
if(NOT MY_CUSTOM_LIB)
    message(FATAL_ERROR "自定义库 libexample.so 未在 /usr/lib 中找到！")
endif()

# 编译器参数设置：开启所有警告 (-Wall)，让代码更严谨
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies 加载 ROS 2 和 Qt6 的核心组件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core) # 如果需要界面，增加 Widgets
# 开启 Qt 的“三自动”功能（关键！）
# 它们会自动处理 Qt 特有的信号槽机制 (MOC)、资源文件 (RCC) 和界面文件 (UIC)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 目标 1：基础 MoveIt 测试
add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)


# 目标 2：面向对象的 MoveIt 测试
add_executable(test_moveit_oop src/command_temple.cpp)
ament_target_dependencies(test_moveit_oop rclcpp moveit_ros_planning_interface)

# 目标 3：核心测试程序 (使用了 Qt 和自定义 .so)
add_executable(test_arm src/test_arm.cpp)
# 5. 包含头文件路径
target_include_directories(test_arm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# 链接库文件 (手动链接)
# 为什么要用 target_link_libraries 而不是上面的 ament 命令？
# 因为 Qt6 和你的自定义 .so 不属于标准的 ROS 2 插件，需要手动指定链接。
target_link_libraries(test_arm
  Qt6::Core
  ${MY_CUSTOM_LIB}  # 链接刚才找到的库路径
)

# 将生成的可执行文件安装到包的 lib 目录下
# 这样你才能通过 'ros2 run my_robot_commander_cpp test_arm' 启动它
install(TARGETS
  test_moveit_oop
  test_moveit
  test_arm
  DESTINATION lib/${PROJECT_NAME}/)

  # 告诉 ament 这是一份标准的 ROS 2 包说明书，负责生成环境勾连文件
ament_package()

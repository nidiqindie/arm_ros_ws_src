# 最低 CMake 版本要求（ROS 2 推荐 3.5 及以上，这里用 3.8 更兼容）
cmake_minimum_required(VERSION 3.8)

# 项目名称（必须与 package.xml 中的 name 一致，建议已修改为不冲突的名称，如 my_urdf_description）
project(my_robot_description)

# 可选：添加编译警告选项（增强代码规范性）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 核心构建依赖（ament_cmake 替代 catkin）
find_package(ament_cmake REQUIRED)

# 查找其他依赖（根据 package.xml 中的依赖添加，这里匹配之前修改的 package.xml）
# find_package(launch REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)


# 安装资源文件（ROS 2 中共享文件安装到 share/${PROJECT_NAME} 路径下）
# 替换原 CATKIN_PACKAGE_SHARE_DESTINATION 为 ROS 2 的标准路径
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# ROS 2 必须的打包指令（替代 catkin_package()）
ament_package()

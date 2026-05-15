cmake_minimum_required(VERSION 2.8.3)
project(orne_box_navigation_executor)

## ---------------------------------------------------------
## 1. 依存パッケージの統合
## ---------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  # 既存の依存
  move_base_msgs
  geometry_msgs
  move_base
  roscpp
  actionlib_msgs
  actionlib
  tf
  std_msgs
  # 新しいプランナー用の依存（追加）
  nav_core
  costmap_2d
  pluginlib
)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

## ---------------------------------------------------------
## 2. パッケージ設定
## ---------------------------------------------------------
catkin_package(
  INCLUDE_DIRS include
  # 既存のライブラリがある場合はそれも残す（もしあれば）
  LIBRARIES ${PROJECT_NAME} straight_line_planner
)

## ---------------------------------------------------------
## 3. インクルードパス
## ---------------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

## ---------------------------------------------------------
## 4. ビルド指示（ここから追加）
## ---------------------------------------------------------

# 新しいプランナーをライブラリとしてビルド
add_library(straight_line_planner
  src/straight_line_planner.cpp
)

# ライブラリに必要なものをリンク
target_link_libraries(straight_line_planner
  ${catkin_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
)



#[[cmake_minimum_required(VERSION 2.8.3)
project(orne_box_navigation_executor)

find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  geometry_msgs
  move_base
  roscpp
  actionlib_msgs
  actionlib
  tf
  std_msgs
)

find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  #include 
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)]]

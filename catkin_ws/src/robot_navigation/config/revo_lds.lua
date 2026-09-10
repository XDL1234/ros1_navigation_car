-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"  -- 包含地图构建器配置
include "trajectory_builder.lua"  -- 包含轨迹构建器配置

options = {
  -- 核心组件配置
  map_builder = MAP_BUILDER,  -- 地图构建器实例
  trajectory_builder = TRAJECTORY_BUILDER,  -- 轨迹构建器实例
  
  -- 坐标系设置
  map_frame = "map",  -- 地图坐标系名称
  tracking_frame = "base_footprint",  -- 传感器数据对齐的坐标系（机器人基座）
  published_frame = "base_footprint",  -- 发布位姿的坐标系
  odom_frame = "odom_combined",  -- 里程计坐标系名称
  provide_odom_frame = true,  -- 是否发布map到odom的tf变换
  publish_frame_projected_to_2d = false,  -- 是否将3D位姿投影到2D
  
  -- 传感器使用配置
  use_odometry = true,  -- 是否使用里程计数据
  use_nav_sat = false,  -- 是否使用GPS数据
  use_landmarks = false,  -- 是否使用路标数据
  
  -- 激光雷达配置
  num_laser_scans = 1,  -- 使用的单线激光话题数量
  num_multi_echo_laser_scans = 0,  -- 使用的多回波激光话题数量
  num_subdivisions_per_laser_scan = 1,  -- 每条激光扫描细分的次数
  num_point_clouds = 0,  -- 使用的点云话题数量
  
  -- 系统参数
  lookup_transform_timeout_sec = 1.0,  -- TF查找超时时间(秒)
  submap_publish_period_sec = 0.3,  -- 子图发布周期(秒)
  pose_publish_period_sec = 5e-3,  -- 位姿发布周期(秒)(200Hz)
  trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期(秒)(~33Hz)
  
  -- 传感器采样率
  rangefinder_sampling_ratio = 1.,  -- 测距仪(激光)采样率(1=100%)
  odometry_sampling_ratio = 1.,  -- 里程计采样率
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定坐标系位姿采样率
  imu_sampling_ratio = 1.,  -- IMU采样率
  landmarks_sampling_ratio = 1.,  -- 路标采样率
}

-- 2D建图配置
MAP_BUILDER.use_trajectory_builder_2d = true  -- 启用2D轨迹构建器

-- 2D轨迹构建器参数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- 每个子图包含的扫描次数
TRAJECTORY_BUILDER_2D.min_range = 0.3  -- 最小有效测距(m)，过滤近距离噪声
TRAJECTORY_BUILDER_2D.max_range = 8.  -- 最大有效测距(m)，过滤远距离噪声
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.  -- 无效数据射线长度(m)
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 是否使用IMU数据（2D建图通常禁用）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 启用在线相关扫描匹配

-- 实时相关扫描匹配器参数
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1  -- 线性搜索窗口(m)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.  -- 平移变化代价权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1  -- 旋转变化代价权重

-- 位姿图优化参数
POSE_GRAPH.optimization_problem.huber_scale = 1e2  -- Huber损失函数比例因子（鲁棒优化）
POSE_GRAPH.optimize_every_n_nodes = 35  -- 每N个节点执行一次全局优化
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 约束匹配最小得分阈值(0-1)

return options
from path_planner import UAVPathPlannerComplete

def main():
    # 定义障碍物
    obstacles = [
        [(-0.6, 1), (-0.6, 10), (0.6, 10), (0.6, 1)],
        [(-0.6, -1), (0.6, -1), (0.6, -10), (-0.6, -10)]
    ]
    
    # 定义无人机数据
    uav_data = [
        {'id': 'uav_1', 'start': [-1.28, -0.69], 'goal': [2.74, 1.53]},
        {'id': 'uav_2', 'start': [1.74, 1.53], 'goal': [-1.28, -0.69]},
        {'id': 'uav_3', 'start': [1.3165, -0.882], 'goal': [-1.95, 1.23]},
        {'id': 'uav_4', 'start': [3, 1], 'goal': [-4, 0.5]},
    ]
    
    # 创建完整路径规划系统
    planner = UAVPathPlannerComplete(
        obstacles=obstacles,
        num_particles=15,
        waypoints=10,
        max_iterations=300,
        min_distance=1.0,
        enable_optimization=True
    )
    
    # 执行完整规划流程
    result = planner.plan_and_optimize_paths(uav_data)
    
    # 打印结果
    print(f"规划成功: {result['success']}")
    print(f"总计算时间: {result['computation_time']:.2f} 秒")
    
    if result['success']:
        print(f"PSO迭代次数: {result['planning_result']['iterations']}")
        print(f"整体压缩率: {result['statistics']['overall_compression_rate']:.1f}%")
        print(f"总航点数: {result['statistics']['total_original_waypoints']} -> {result['statistics']['total_final_waypoints']}")
        
        print("\n各无人机最终路径:")
        for uav_id, path_info in result['final_paths'].items():
            print(f"\n{uav_id}:")
            print(f"  路径点数: {path_info['waypoint_count']}")
            print(f"  时间索引: {path_info['time_indices']}")
            print(f"  压缩率: {result['statistics']['per_uav_stats'][uav_id]['compression_rate']:.1f}%")
            print(f"  路径长度: {result['statistics']['per_uav_stats'][uav_id]['path_length']:.2f}")
            
            path = path_info['path']
            if len(path) <= 6:
                print(f"  路径点: {path}")
            else:
                print(f"  路径点: {path[:3]} ... {path[-3:]}")
        
        validation = planner.validate_paths(result)
        print(f"\n路径验证: {'通过' if validation['valid'] else '失败'}")
        if not validation['collision_free']:
            print("⚠️ 存在距离不足或碰撞问题")

if __name__ == "__main__":
    main()
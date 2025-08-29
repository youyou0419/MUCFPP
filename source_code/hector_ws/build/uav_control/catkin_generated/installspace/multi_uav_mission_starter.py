#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
import roslaunch
import subprocess

class MultiUAVMissionStarter:
    def __init__(self):
        # 初始化节点
        rospy.init_node('multi_uav_mission_starter')
        
        # 获取参数，确定有多少架无人机
        self.uav_count = rospy.get_param('~uav_count', 4)  # 默认为3架无人机
        
        # 创建服务
        self.service = rospy.Service('start_all_missions', Empty, self.start_all_missions_callback)
        
        rospy.loginfo("Multi-UAV mission starter service is ready.")
    
    def start_all_missions_callback(self, req):
        rospy.loginfo("Received request to start all UAV missions")
        
        # 为每架无人机调用start_mission服务
        for i in range(1, self.uav_count + 1):
            uav_name = 'uav{}'.format(i)
            service_name = '/{}/start_mission'.format(uav_name)
            
            try:
                rospy.loginfo("Calling service: {}".format(service_name))
                rospy.wait_for_service(service_name, timeout=2.0)
                start_mission = rospy.ServiceProxy(service_name, Empty)
                start_mission()
                rospy.loginfo("Successfully started mission for {}".format(uav_name))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed for {}: {}".format(uav_name, str(e)))
            except rospy.ROSException as e:
                rospy.logerr("Service {} not available: {}".format(service_name, str(e)))
        
        return EmptyResponse()

if __name__ == "__main__":
    try:
        starter = MultiUAVMissionStarter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
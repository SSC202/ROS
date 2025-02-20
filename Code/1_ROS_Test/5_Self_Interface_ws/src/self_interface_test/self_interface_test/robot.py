from self_interface.msg import RobotStatus
import math
from time import sleep

class Robot():
    def __init__(self):
        """
            机器人属性
            1.当前状态
            2.当前位置
            3.目标位置
        """
        self.status = RobotStatus.STATUS_STOPPING
        self.current_pose = 0.0
        self.target_pose = 0.0

    """
        返回当前状态
    """
    def get_status(self):
        return self.status
    
    """
        返回当前位置
    """
    def get_current_pose(self):
        return self.current_pose
    
    """
        接受指令并更新目标位置，开始移动并随时返回当前位置
    """
    def move_robot(self,distance):
        # 更新状态
        self.status = RobotStatus.STATUS_MOVING
        # 更新目标位置
        self.target_pose += distance

        # 开始移动机器人
        ## 如果未到达目标点
        while math.fabs(self.current_pose - self.target_pose) > 0.01:
            ## 移动剩余距离的0.1倍
            step = distance / math.fabs(distance) * math.fabs(self.current_pose - self.target_pose) * 0.1
            self.current_pose += step
            print(f"Current Pose:{self.current_pose}")
            sleep(1)
        
        self.status = RobotStatus.STATUS_STOPPING
        return self.current_pose
    




    

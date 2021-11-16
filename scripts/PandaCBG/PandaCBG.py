from CLG_calibrate_loop import calibrate_loop
import franka_interface
from moveit_commander.move_group import MoveGroupCommander
import rospy
from franka_interface import ArmInterface
from franka_moveit import PandaMoveGroupInterface
from franka_moveit.utils import create_pose_msg
from cable_loop_gripper.msg import CLGcommand,CLGstatus
from cable_loop_gripper.srv import CLGvision_loopRadius,CLGvision_vialXYdistance
import numpy as np
from math import sqrt,cos,sin,pi

class PandaCBG:
    def __init__(self):
        
        rospy.init_node('panda_arm_experiment')
        self.pub = rospy.Publisher("/CLG_command", CLGcommand, queue_size=10)
        self.sub = rospy.Subscriber("/CLG_status", CLGstatus, self.CLGcallback)
        self.arm = ArmInterface()
        self.arm_movegroup = PandaMoveGroupInterface()
        self.rate = rospy.Rate(2)
        self.camera_rate = rospy.Rate(1)
        self.proxy_xy = rospy.ServiceProxy('/CLG_control_node/CLG_vision_vial_xy_distance',CLGvision_vialXYdistance)
        self.proxy_cl = rospy.ServiceProxy('/CLG_control_node/CLG_vision_loop_radius',CLGvision_loopRadius)
        self.default_ori = start_ori = self.arm.get_flange_pose()[1]
        self.default_ori.w = 0.0000
        self.default_ori.x = -0.924
        self.default_ori.y = 0.3826
        self.default_ori.z = 0.0000

        self._current_radius = 20.0
        self._current_force = 0.0

    def CLGcallback(self, data):
        self._current_radius = float(data.current_loop_radius)
        self._current_force = float(data.current_force)
    
    def GripRoutine(self, force=10.0):
        msg = CLGcommand()
        value = float(force)            
        msg.requested_loop_radius= 0
        msg.requested_force = value
        msg.control_mode = False
        self.pub.publish(msg)
        while self._current_force < 19:
            self.rate.sleep()

    def MoveLoopRoutine(self, radius=10.0):
        msg = CLGcommand()
        value = float(radius)            
        msg.requested_loop_radius= value
        msg.requested_force = 0
        msg.control_mode = True
        self.pub.publish(msg)
        while (radius-0.2) <= self._current_radius <= (radius+0.2):
            self.rate.sleep()

    def FindVialError(self):
        response = self.proxy_xy(True)
        print(response)
        y = -response.x_distance
        x = -response.y_distance
        r = response.vial_radius
        return float(x), float(y), float(r)

    def FindLoopRadius(self):
        r = float(self.proxy_cl(True).loop_radius)
        return r*10

    def MoveToCartesianPosInterpolated(self, target):
        start_pos = self.arm.get_flange_pose()[0]
        x_delta = target[0]-start_pos[0]
        y_delta = target[1]-start_pos[1]
        z_delta = target[2]-start_pos[2]
        n_steps = int(sqrt(x_delta**2+y_delta**2+z_delta**2)*10000)
        x_step = x_delta/n_steps
        y_step = y_delta/n_steps
        z_step = z_delta/n_steps
        plan = []
        for step in range(n_steps):
            delta_step = [x_step*step, y_step*step, z_step*step]
            interpolated_pos = start_pos + delta_step
            plan.append(create_pose_msg(interpolated_pos,self.default_ori))
        moveit_plan = self.arm_movegroup.plan_cartesian_path(plan)[0]
        self.arm_movegroup.execute_plan(moveit_plan)

    def MoveCartesianDeltaInterpolated(self, delta):
        start_pos = self.arm.get_flange_pose()[0]
        n_steps = int(sqrt(delta[0]**2+delta[1]**2+delta[2]**2)*10000)
        x_step = delta[0]/n_steps
        y_step = delta[1]/n_steps
        z_step = delta[2]/n_steps
        plan = []
        for step in range(n_steps):
            delta_step = [x_step*step, y_step*step, z_step*step]
            interpolated_pos = start_pos + delta_step
            plan.append(create_pose_msg(interpolated_pos,self.default_ori))
        moveit_plan = self.arm_movegroup.plan_cartesian_path(plan)[0]
        self.arm_movegroup.execute_plan(moveit_plan)

    def InitArm(self, straight=True):
        self.arm.move_to_neutral()
        start_pos = self.arm.endpoint_pose()['position']
        start_ori = self.arm.endpoint_pose()['orientation']
        if straight:
            start_ori.w = 0.0
            start_ori.x = -1.0
            start_ori.y = 0.0
            start_ori.z = 0.0
        else:
            start_ori.w = 0.0
            start_ori.x = 0.9238
            start_ori.y = 0.3826
            start_ori.z = 0.0
        self.arm.move_to_cartesian_pose(start_pos, start_ori)
        start_ori = self.arm.get_flange_pose()[1]
    
    def InitGripper(self):
        calibrate_loop(10)
        self.MoveLoopRoutine(20)

    def ServoToVial(self, straight=True, r_offset=5.5):
        self.camera_rate.sleep()
        x, y, r = self.FindVialError()
        r_open = r + r_offset
        if straight:
            self.MoveCartesianDeltaInterpolated([(x)/1000, (y+r+1)/1000, 0])
        else:
            self.MoveCartesianDeltaInterpolated([0, 0, 0])
        self.MoveLoopRoutine(r_open)
        return r_open

    def PickupVial(self, vial_position, drop, force=50, straight=True):
        self.MoveToCartesianPosInterpolated([vial_position[0], vial_position[1]+0.002, vial_position[2]])
        r_open = self.ServoToVial(straight)
        self.MoveCartesianDeltaInterpolated([0,0,-drop])
        if straight:
            self.MoveCartesianDeltaInterpolated([0,-0.001,0])
        else:
            self.MoveCartesianDeltaInterpolated(-0.001/sqrt(2),0.001/sqrt(2))
        self.GripRoutine(50)
        self.MoveCartesianDeltaInterpolated([0,0,2*drop])
        return r_open

    def DepositVial(self, vial_position, drop, loop_release=15):
        self.MoveToCartesianPosInterpolated([vial_position[0], vial_position[1], vial_position[2]+drop])
        self.MoveCartesianDeltaInterpolated([0,0,-1.9*drop])
        self.MoveLoopRoutine(loop_release+1)
        self.rate.sleep()
        self.rate.sleep()
        self.MoveCartesianDeltaInterpolated([0,0.001,0])
        self.MoveCartesianDeltaInterpolated([0,0,drop])
        self.MoveLoopRoutine(20)
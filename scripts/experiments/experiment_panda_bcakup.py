#!/usr/bin/env python3

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

class VialTray:
    def __init__(self, rows=3, cols=3, vial_r=0.012, vial_spacing=0.026, tray_origin=(0.2,0.2,0.0), z_drop=0.05):
        self._rows = rows
        self._cols = cols
        self._n_vials = self._rows*self._cols

        self._vial_r = vial_r
        self._vial_spacing = vial_spacing
        self._tray_origin = np.array(tray_origin)

        self._z_drop = z_drop

        self._vial_positions = []
        for i in range(self._rows):
            for j in range(self._cols):
                delta = np.array([i*self._vial_spacing, -j*self._vial_spacing, 0.0])
                self._vial_positions.append(self._tray_origin+delta)
        
    def vial_positions(self, i=None):
        if i==None:
            return self._vial_positions
        else:
            return self._vial_positions[i]

    def n_vials(self):
        return self._n_vials

def experiment():

    rospy.init_node('panda_arm_experiment')
    pub = rospy.Publisher("/CLG_command", CLGcommand, queue_size=10)
    arm = ArmInterface()
    arm_movegroup = PandaMoveGroupInterface()
    rate = rospy.Rate(0.3)
    proxy_xy = rospy.ServiceProxy('/CLG_control_node/CLG_vision_vial_xy_distance',CLGvision_vialXYdistance)
    proxy_cl = rospy.ServiceProxy('/CLG_control_node/CLG_vision_loop_radius',CLGvision_loopRadius)
    default_ori = start_ori = arm.get_flange_pose()[1]
    default_ori.w = 0.0000
    default_ori.x = -0.924
    default_ori.y = 0.3826
    default_ori.z = 0.0000

    def GripRoutine(force=10.0):
        msg = CLGcommand()
        value = float(force)            
        msg.requested_loop_radius= 0
        msg.requested_force = value
        msg.control_mode = False
        pub.publish(msg)

    def MoveLoopRoutine(radius=10.0):
        msg = CLGcommand()
        value = float(radius)            
        msg.requested_loop_radius= value
        msg.requested_force = 0
        msg.control_mode = True
        pub.publish(msg)

    def FindVialError():
        response = proxy_xy(True)
        print(response)
        y = -response.x_distance
        x = -response.y_distance
        r = response.vial_radius
        return float(x), float(y), float(r)

    def FindLoopRadius():
        r = float(proxy_cl(True).loop_radius)
        return r*10

    def MoveCartesianDelta(delta):
        start_pose = arm.endpoint_pose()['position']
        new_pose = start_pose + delta
        arm.move_to_cartesian_pose(new_pose)

    def MoveToCartesianPosInterpolated(target):
        start_pos = arm.get_flange_pose()[0]
        start_ori = arm.get_flange_pose()[1]
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
            plan.append(create_pose_msg(interpolated_pos,default_ori))
        moveit_plan = arm_movegroup.plan_cartesian_path(plan)[0]
        arm_movegroup.execute_plan(moveit_plan)

    def MoveCartesianDeltaInterpolated(delta):
        start_pos = arm.get_flange_pose()[0]
        start_ori = arm.get_flange_pose()[1]
        n_steps = int(sqrt(delta[0]**2+delta[1]**2+delta[2]**2)*10000)
        x_step = delta[0]/n_steps
        y_step = delta[1]/n_steps
        z_step = delta[2]/n_steps
        plan = []
        for step in range(n_steps):
            delta_step = [x_step*step, y_step*step, z_step*step]
            interpolated_pos = start_pos + delta_step
            plan.append(create_pose_msg(interpolated_pos,default_ori))
        moveit_plan = arm_movegroup.plan_cartesian_path(plan)[0]
        arm_movegroup.execute_plan(moveit_plan)

    def InitArmGripper(straight=True):
        arm.move_to_neutral()
        #start_pos = arm.get_flange_pose()[0]
        #start_ori = arm.get_flange_pose()[1]
        start_pos = arm.endpoint_pose()['position']
        start_ori = arm.endpoint_pose()['orientation']
        
        if straight:
            #start_ori.w = 0.0000
            #start_ori.x = -0.924
            #start_ori.y = 0.3826
            #start_ori.z = 0.0000
            start_ori.w = 0.0
            start_ori.x = -1.0
            start_ori.y = 0.0
            start_ori.z = 0.0
        else:
            start_ori.w = 0.0
            start_ori.x = 0.9238
            start_ori.y = 0.3826
            start_ori.z = 0.0
        #plan = []
        #plan.append(create_pose_msg(start_pos,start_ori))
        #moveit_plan = arm_movegroup.plan_cartesian_path(plan)[0]
        #arm_movegroup.execute_plan(moveit_plan)
        #print(plan)
        #arm_movegroup.execute_plan(moveit_plan)
        #start_pose = create_pose_msg([0.3,0.3,0.3],start_ori)
        #arm_movegroup.go_to_cartesian_pose(start_pose)
        arm.move_to_cartesian_pose(start_pos, start_ori)
        start_ori = arm.get_flange_pose()[1]
        print(start_ori.z)
        calibrate_loop(10)
        MoveLoopRoutine(20)

    def ServoToVial(straight=True, r_offset=5.5):
        rate.sleep()
        x, y, r = FindVialError()
        r_open = r + r_offset
        if straight:
            MoveCartesianDeltaInterpolated([(x)/1000, (y+r+1)/1000, 0])
        else:
            MoveCartesianDeltaInterpolated([0, 0, 0])
        MoveLoopRoutine(r_open)
        return r_open

    def PickupVial(vial_position, drop, force=50, straight=True):
        MoveToCartesianPosInterpolated([vial_position[0], vial_position[1]+0.002, vial_position[2]])
        r_open = ServoToVial(straight)
        rate.sleep()
        MoveCartesianDeltaInterpolated([0,0,-drop])
        if straight:
            MoveCartesianDeltaInterpolated([0,-0.001,0])
        else:
            MoveCartesianDeltaInterpolated(-0.001/sqrt(2),0.001/sqrt(2))
        GripRoutine(50)
        rate.sleep()
        MoveCartesianDelta([0,0,2*drop])
        return r_open

    def DepositVial(vial_position, drop, loop_release=15):
        MoveToCartesianPosInterpolated([vial_position[0], vial_position[1], vial_position[2]+drop])
        MoveCartesianDelta([0,0,-1.9*drop])
        MoveLoopRoutine(loop_release+1)
        rate.sleep()
        rate.sleep()
        MoveCartesianDeltaInterpolated([0,0.001,0])
        rate.sleep()
        MoveCartesianDeltaInterpolated([0,0,drop])
        MoveLoopRoutine(20)

    def Tray2Tray():
        tray_1_pos = [0.52, 0.05, 0.227]
        tray_2_pos = [0.52, -0.15, 0.227]
        tray_1 = VialTray(3,2,0.012,0.026,tray_1_pos,0.042)
        tray_2 = VialTray(3,2,0.012,0.026,tray_2_pos,0.042)
        InitArmGripper()
        r = PickupVial(tray_1.vial_positions(0), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(1), tray_1._z_drop, r)
        r = PickupVial(tray_1.vial_positions(2), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(3), tray_1._z_drop, r)
        r = PickupVial(tray_1.vial_positions(4), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(5), tray_1._z_drop, r)
        r = PickupVial(tray_1.vial_positions(1), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(0), tray_1._z_drop, r)
        r = PickupVial(tray_1.vial_positions(3), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(2), tray_1._z_drop, r)
        r = PickupVial(tray_1.vial_positions(5), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(4), tray_1._z_drop, r)
        arm.move_to_neutral()

    def Tray2TrayInverse():
        tray_2_pos = [0.52, 0.05, 0.227]
        tray_1_pos = [0.52, -0.15, 0.227]
        tray_1 = VialTray(3,2,0.012,0.026,tray_1_pos,0.04)
        tray_2 = VialTray(3,2,0.012,0.026,tray_2_pos,0.04)
        InitArmGripper()
        r = PickupVial(tray_1.vial_positions(0), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(1), tray_1._z_drop, r)
        r = PickupVial(tray_1.vial_positions(2), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(3), tray_1._z_drop, r)
        r = PickupVial(tray_1.vial_positions(4), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(5), tray_1._z_drop, r)
        r = PickupVial(tray_1.vial_positions(1), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(0), tray_1._z_drop, r)
        r = PickupVial(tray_1.vial_positions(3), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(2), tray_1._z_drop, r)
        r = PickupVial(tray_1.vial_positions(5), tray_1._z_drop)
        DepositVial(tray_2.vial_positions(4), tray_1._z_drop, r)
        arm.move_to_neutral()
    
    i = 0
    while 1:
        i = i+1
        print("iteration ",i)
        Tray2TrayInverse()
        i = i+1
        print("iteration ",i)
        Tray2Tray()

if __name__ == "__main__":
    experiment()
    
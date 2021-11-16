#!/usr/bin/env python3

from PandaCBG.tray import VialTray as Tray
from PandaCBG.PandaCBG import PandaCBG  

arm = PandaCBG()

tray_1_pos = [0.52, 0.0525, 0.227]
tray_2_pos = [0.52, -0.15, 0.227]
tray_1 = Tray(3,2,0.012,0.026,tray_1_pos,0.042)
tray_2 = Tray(3,2,0.012,0.026,tray_2_pos,0.042)

def Tray2Tray():
        arm.InitArm()
        arm.InitGripper()
        r = arm.PickupVial(tray_1.vial_positions(0), tray_1._z_drop)
        arm.DepositVial(tray_2.vial_positions(1), tray_1._z_drop, r)
        r = arm.PickupVial(tray_1.vial_positions(2), tray_1._z_drop)
        arm.DepositVial(tray_2.vial_positions(3), tray_1._z_drop, r)
        r = arm.PickupVial(tray_1.vial_positions(4), tray_1._z_drop)
        arm.DepositVial(tray_2.vial_positions(5), tray_1._z_drop, r)
        r = arm.PickupVial(tray_1.vial_positions(1), tray_1._z_drop)
        arm.DepositVial(tray_2.vial_positions(0), tray_1._z_drop, r)
        r = arm.PickupVial(tray_1.vial_positions(3), tray_1._z_drop)
        arm.DepositVial(tray_2.vial_positions(2), tray_1._z_drop, r)
        r = arm.PickupVial(tray_1.vial_positions(5), tray_1._z_drop)
        arm.DepositVial(tray_2.vial_positions(4), tray_1._z_drop, r)

def Tray2TrayInverse():
        arm.InitArm()
        arm.InitGripper()
        r = arm.PickupVial(tray_2.vial_positions(0), tray_1._z_drop)
        arm.DepositVial(tray_1.vial_positions(1), tray_1._z_drop, r)
        r = arm.PickupVial(tray_2.vial_positions(2), tray_1._z_drop)
        arm.DepositVial(tray_1.vial_positions(3), tray_1._z_drop, r)
        r = arm.PickupVial(tray_2.vial_positions(4), tray_1._z_drop)
        arm.DepositVial(tray_1.vial_positions(5), tray_1._z_drop, r)
        r = arm.PickupVial(tray_2.vial_positions(1), tray_1._z_drop)
        arm.DepositVial(tray_1.vial_positions(0), tray_1._z_drop, r)
        r = arm.PickupVial(tray_2.vial_positions(3), tray_1._z_drop)
        arm.DepositVial(tray_1.vial_positions(2), tray_1._z_drop, r)
        r = arm.PickupVial(tray_2.vial_positions(5), tray_1._z_drop)
        arm.DepositVial(tray_1.vial_positions(4), tray_1._z_drop, r)

def experiment():
    i = 0
    while 1:
        i = i+1
        print("iteration ",i)
        Tray2Tray()
        i = i+1
        print("iteration ",i)
        Tray2TrayInverse()

if __name__ == "__main__":
    experiment()

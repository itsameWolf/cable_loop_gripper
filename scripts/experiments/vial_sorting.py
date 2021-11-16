#!/usr/bin/env python3

from PandaCBG.tray import VialTray as Tray
from PandaCBG.PandaCBG import PandaCBG 

arm = PandaCBG()

tray_17mm = Tray(rows=3,cols=1,vial_r=12,vial_spacing=30,tray_origin=[0.52,0.2,0.235],z_drop=6)
tray_24mm = Tray(rows=3,cols=1,vial_r=12,vial_spacing=30,tray_origin=[0.49,0.2,0.235],z_drop=6)
tray_28mm = Tray(rows=3,cols=1,vial_r=14,vial_spacing=30,tray_origin=[0.46,0.2,0.235],z_drop=6)

vial_line = Tray(rows=9,cols=1,vial_r=15,vial_spacing=30,tray_origin=[0.52,0.2,0.235],z_drop=6)

def VialSorting():
    position_17mm = 0
    position_24mm = 0
    position_28mm = 0
    for line_position in range(9):
        r = arm.PickupVial(vial_line.vial_positions(line_position))
        if (r-5.5) < 17:
            arm.DepositVial(tray_17mm.vial_positions(position_17mm))
            position_17mm = position_17mm + 1
        elif 24 > (r-5.5) > 17:
            arm.DepositVial(tray_24mm.vial_positions(position_24mm))
            position_24mm = position_24mm + 1
        else:
            arm.DepositVial(tray_28mm.vial_positions(position_24mm))
            position_28mm = position_28mm + 1

if __name__ == "__main__":
    VialSorting()
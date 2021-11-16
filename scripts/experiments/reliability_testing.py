#!/usr/bin/env python3

from PandaCBG.tray import VialTray as Tray
from PandaCBG.PandaCBG import PandaCBG 
import sys 

RED   = "\033[1;31m"  
BLUE  = "\033[1;34m"
CYAN  = "\033[1;36m"
GREEN = "\033[0;32m"
RESET = "\033[0;0m"
BOLD    = "\033[;1m"
REVERSE = "\033[;7m"

arm = PandaCBG()

tray_1_17mm = Tray(rows=3,cols=1,vial_r=12,vial_spacing=30,tray_origin=[0.52,0.2,0.235],z_drop=6)
tray_1_24mm = Tray(rows=3,cols=1,vial_r=12,vial_spacing=30,tray_origin=[0.49,0.2,0.235],z_drop=6)
tray_1_28mm = Tray(rows=3,cols=1,vial_r=14,vial_spacing=30,tray_origin=[0.46,0.2,0.235],z_drop=6)

tray_1 = list(tray_1_17mm,tray_1_24mm,tray_1_28mm)

tray_2_17mm = Tray(rows=3,cols=1,vial_r=12,vial_spacing=30,tray_origin=[0.52,-0.2,0.235],z_drop=6)
tray_2_24mm = Tray(rows=3,cols=1,vial_r=12,vial_spacing=30,tray_origin=[0.49,-0.2,0.235],z_drop=6)
tray_2_28mm = Tray(rows=3,cols=1,vial_r=14,vial_spacing=30,tray_origin=[0.46,-0.2,0.235],z_drop=6)

tray_2 = list(tray_2_17mm,tray_2_24mm,tray_2_28mm)

vial_n = 0

def PrintVialNumber():
    sys.stdout.write(BOLD)
    print(str(vial_n) + " vials moved")
    sys.stdout.write(RESET)

def Tray2Tray(origin, target):
    for i in range(3):
        for j in range(3):
            arm.PickupVial(origin[j].vial_positions(i),origin[j]._z_drop)
            arm.DepositVial(origin[j].vial_positions(2-i),origin[j]._z_drop)
            vial_n = vial_n + 1
            PrintVialNumber()

if __name__ == "__main__":
    while vial_n < 720:
        Tray2Tray(tray_1,tray_2)
        Tray2Tray(tray_2,tray_1)
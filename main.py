from pybricks.hubs import *
from pybricks.pupdevices import *
from pybricks.parameters import *
from pybricks.tools import *
from umath import *
 
# --- HUB és motorok ---
hub = PrimeHub()
bal  = Motor(Port.B)
jobb = Motor(Port.F, Direction.COUNTERCLOCKWISE)
 
def clamp(x, a, b):
    return max(a, min(b, x))
 
def avg_angle_deg():
    return (abs(bal.angle()) + abs(jobb.angle())) / 2
 
 
hub.imu.reset_heading(0)
irany = 0
def forward(distance_mm, min_speed=40, easein=40, korekcio=0.01, max_speed = 700, easeout=80):
    global irany
    bal.reset_angle(0)
    jobb.reset_angle(0)
    distance_mm = distance_mm / 0.489
    easein /= 0.489
    easeout /= 0.489
    while True:
        dist_traveled = (bal.angle()+jobb.angle()) / 2
        curpos = distance_mm - dist_traveled
        sign = curpos/abs(curpos)
        dist_traveled = abs(dist_traveled)
        curpos = abs(curpos)
        print(dist_traveled, curpos)
        if abs(curpos) < 3:
            break
        if curpos < easeout :
            ratio = curpos / easeout
            curspeed = max(ratio * max_speed, min_speed) * sign
        elif dist_traveled < easein:
            ratio = dist_traveled / easein
            curspeed = max(ratio * max_speed, min_speed) * sign
        else:
            curspeed = max_speed * sign
        iranyelteres = (irany - hub.imu.heading()) * korekcio
        korekciomertek = curspeed * iranyelteres * sign
        bal.run (curspeed - korekciomertek)
        jobb.run(curspeed + korekciomertek)
    bal.hold()
    jobb.hold()



def clamp(x, a, b):
    return max(a, min(b, x))
 
# -------- STABIL FORDULÁS --------
def turn(angle_deg, max_speed=360, easeout=80, min_speed=50):
 
    alap_fok = hub.imu.heading()
    global irany
    cel_fok = irany+angle_deg
    irany = cel_fok
    cel_fok -= alap_fok
    # print(cel_fok, alap_fok)
    while True:
        angle_turned = hub.imu.heading() - alap_fok
        angle_toturn = cel_fok - angle_turned
        # print(angle_turned, angle_toturn)
        sign = angle_toturn/abs(angle_toturn)
        angle_toturn = abs(angle_toturn)
        if angle_toturn <= 0.5:
            break
        
        if angle_toturn < easeout :
            ratio = angle_toturn / easeout
            curspeed = max(ratio * max_speed, min_speed) * sign
        else:
            curspeed = max_speed * sign
        
        bal.run(-curspeed)
        jobb.run(curspeed)
    bal.hold()
    jobb.hold()
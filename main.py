from pybricks.hubs import * #beimportálja az agyat
from pybricks.pupdevices import * #beimportálja a pupdevicesokat
from pybricks.parameters import * #beimportálja a paramétereket
from pybricks.tools import * #beimportálja a toolsokat


hub = PrimeHub() #az agyat elnevezi hubnak
bal  = Motor(Port.B) #a motort ami a B portba van elnevezzük balnak és óra járásával megegyező irányba forog
jobb = Motor(Port.F, Direction.COUNTERCLOCKWISE) #a motort ami az F portba van elnevezzük balnak és óra járásával ellentétes irányba forog
 
hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
irany = 0 #létrehozunk egy irany nevű változót aminek 0 értéket adunk
def egyenes(egyenes_tavolsag, egyenes_legkisebb_sebesseg=40, egyenes_gyorsitas=40, korekcio=0.01, egyenes_legnagyobb_sebesseg = 700, egyenes_lassitas=80): #létrehozunk egy egyenes nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, az egyenes_tavolsagot, egyenes_lassitast és a egyenes_gyorsitast mm-be adjuk meg, az alap értékek csak átlagban működnek 
    global irany #engedélyezzük a függvénynek az irany változó használatát a függvényen belül
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    egyenes_tavolsag = egyenes_tavolsag / 0.489 #a mm-ben megadott egyenes_tavolsagot átváltjuk motor fokokra, a kerék kerületét elosztjuk 360nal
    egyenes_gyorsitas /= 0.489 #a mm-ben megadott egyenes_gyorsitast átváltjuk motor fokokra, a kerék kerületét elosztjuk 360nal
    egyenes_lassitas /= 0.489 #a mm-ben megadott egyenes_lassitast átváltjuk motor fokokra, a kerék kerületét elosztjuk 360nal
    while True: #elindítunk egy ciklust ami addig fut ameddig le nem állítjuk
        egyenes_megtett_tavolsag = (bal.angle()+jobb.angle()) / 2
        curpos = egyenes_tavolsag - egyenes_megtett_tavolsag
        sign = curpos/abs(curpos)
        egyenes_megtett_tavolsag = abs(egyenes_megtett_tavolsag)
        curpos = abs(curpos)
        print(egyenes_megtett_tavolsag, curpos)
        if abs(curpos) < 3:
            break
        if curpos < egyenes_lassitas :
            ratio = curpos / egyenes_lassitas
            curspeed = max(ratio * egyenes_legnagyobb_sebesseg, egyenes_legkisebb_sebesseg) * sign
        elif egyenes_megtett_tavolsag < egyenes_gyorsitas:
            ratio = egyenes_megtett_tavolsag / egyenes_gyorsitas
            curspeed = max(ratio * egyenes_legnagyobb_sebesseg, egyenes_legkisebb_sebesseg) * sign
        else:
            curspeed = egyenes_legnagyobb_sebesseg * sign
        iranyelteres = (irany - hub.imu.heading()) * korekcio
        korekciomertek = curspeed * iranyelteres * sign
        bal.run (curspeed - korekciomertek)
        jobb.run(curspeed + korekciomertek)
    bal.hold()
    jobb.hold()

 
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
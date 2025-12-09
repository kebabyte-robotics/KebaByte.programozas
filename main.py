from pybricks.hubs import *
from pybricks.pupdevices import *
from pybricks.parameters import *
from pybricks.robotics import *
from pybricks.tools import *
from umath import *
 
hub = PrimeHub() #elnevezzük a PrimeHub()-ot hub változóra
motorB = Motor(Port.E, Direction.COUNTERCLOCKWISE) #elnevezzük a Motort nagy_motorB változóra port, irány
motorJ = Motor(Port.A) #elnevezzük a Motort nagy_motorJ változóra port, (irány nem kell ha CLOCKWISE)
kerek_motorJ = Motor(Port.F, Direction.COUNTERCLOCKWISE) #port, irány
kerek_motorB = Motor(Port.B) #nem kell irány ha clockwise
db = DriveBase(kerek_motorB, kerek_motorJ, wheel_diameter=56, axle_track=87.5)   #bal, jobb, wheeldiameter, axle track
db.settings(
    straight_speed=390,
    straight_acceleration=1466,
    turn_speed=426,
    turn_acceleration=1920
)
hub.system.set_stop_button(Button.BLUETOOTH) #beállítja a stopgombot a bluetooth gombra
kerek_atmero = 43.2 #kerék átmérője                     
kerek_kerulet = pi * kerek_atmero  #a kerékkerületét kiszámítjuk, hogy fok->mm pi*kerék átmérő
egyenes_ero = 5.6 #milyen erősen reagáljon az eltérésre: pl 2 fok az 2*5,6/2=5,6az egyik motor nagyobb másik kisebb                                      
def forward(tavolsag, legkisebb_sebesseg, maximalis_sebesseg): 
    kerek_motorB.reset_angle(0)  #bal motor foka 0               
    kerek_motorJ.reset_angle(0)  #jobb motor foka 0              
    hub.imu.reset_heading(0)   #gyro foka 0 ehhez képest lesz eltérés            
    cel_fok = (360 / kerek_kerulet) * tavolsag #a teljes köből kiszámoljuk 56 az hány fok majd megszolozzuk a távolsággal
    while True: #örökre megy amig nem lépünk ki
        bal_fok = abs(kerek_motorB.angle())  #mennyit fordult a bal motor, kell abs mert lehet -   
        jobb_fok = abs(kerek_motorJ.angle())  #mennyit fordult a jobb motor, kell abs mert lehet -
        atlag_fok = (bal_fok + jobb_fok) / 2 #mennyi az átlaguk, kicsi eltérés miatt
        haladas = atlag_fok / cel_fok #haladás aránya 0-1
        if haladas < 0.3: #ha az arány kisebb 0.3-nál, alsó 30%
            aktualis_sebesseg = legkisebb_sebesseg + (maximalis_sebesseg - legkisebb_sebesseg) * (haladas / 0.3) #alap a legkisebb, megmondja + mennyi, 0-0.3 arány
        elif haladas > 0.75: #ha az arány nagyobb 0.75-nél, felső 25%
            aktualis_sebesseg = legkisebb_sebesseg + (maximalis_sebesseg - legkisebb_sebesseg) * ((1 - haladas) / 0.25) #alap a legkisebb, megmondja + mennyi, lassítás miatt 1, 0.75-1 arány 
        else: #ha 0.3-0.75
            aktualis_sebesseg = maximalis_sebesseg #menjen max sebeséggel
        irany_hiba = hub.imu.heading()     #mennyire fordult el az eredetitől
        korrekcio = irany_hiba * egyenes_ero / 2 #fokot megszorozzuk 5,6 felével, ha minus balra fordult sokat, ha plus akkor jobbra
        kerek_motorB.run(aktualis_sebesseg - korrekcio)  #lelassít a korekcióval
        kerek_motorJ.run(aktualis_sebesseg + korrekcio) #gyorsít a korekcióval
        if atlag_fok >= cel_fok: #azért >=, mert lehet negyed fokkal több
            break            # kilép a ciklusból  

def backward(tavolsag, legkisebb_sebesseg, maximalis_sebesseg): 
    kerek_motorB.reset_angle(0)  #bal motor foka 0               
    kerek_motorJ.reset_angle(0)  #jobb motor foka 0              
    hub.imu.reset_heading(0)   #gyro foka 0 ehhez képest lesz eltérés            
    cel_fok = (360 / kerek_kerulet) * tavolsag #a teljes köből kiszámoljuk 56 az hány fok majd megszolozzuk a távolsággal
    while True: #örökre megy amig nem lépünk ki
        bal_fok = abs(kerek_motorB.angle())  #mennyit fordult a bal motor, kell abs mert lehet -   
        jobb_fok = abs(kerek_motorJ.angle())  #mennyit fordult a jobb motor, kell abs mert lehet -
        atlag_fok = (bal_fok + jobb_fok) / 2 #mennyi az átlaguk, kicsi eltérés miatt
        haladas = atlag_fok / cel_fok #haladás aránya 0-1
        if haladas < 0.3: #ha az arány kisebb 0.3-nál, alsó 30%
            aktualis_sebesseg = legkisebb_sebesseg + (maximalis_sebesseg - legkisebb_sebesseg) * (haladas / 0.3) #alap a legkisebb, megmondja + mennyi, 0-0.3 arány
        elif haladas > 0.75: #ha az arány nagyobb 0.75-nél, felső 25%
            aktualis_sebesseg = legkisebb_sebesseg + (maximalis_sebesseg - legkisebb_sebesseg) * ((1 - haladas) / 0.25) #alap a legkisebb, megmondja + mennyi, lassítás miatt 1, 0.75-1 arány 
        else: #ha 0.3-0.75
            aktualis_sebesseg = maximalis_sebesseg #menjen max sebeséggel
        irany_hiba = hub.imu.heading()     #mennyire fordult el az eredetitől
        korrekcio = irany_hiba * egyenes_ero / 2 #fokot megszorozzuk 5,6 felével
        kerek_motorB.run(-aktualis_sebesseg + korrekcio)  #lelassít a korekcióval, minusz mert hátra megy
        kerek_motorJ.run(-aktualis_sebesseg - korrekcio) #gyorsít a korekcióval, minusz mert előre megy
        if atlag_fok >= cel_fok: #azért >=, mert lehet negyed fokkal több
            break            #kilép a ciklusból  
def turn(szog, max_sebesseg=300, min_sebesseg=80, ero=4):
    hub.imu.reset_heading(0)  #gyro foka legyen 0, ez adja az eltérést
    while True: #örök ciklus amig nem breakelünk
        aktualis_szog = hub.imu.heading()   #a mostani szög legyen egyenlő a gyroval
        szog_tavolsag = szog - aktualis_szog   #mennyire vagyunk még messze a céltól
        if abs(szog_tavolsag) < 0.5:  #ha fél foknál közelebb van
            break   #lépjen ki
        # arányos lassítás a célhoz közeledve
        sebesseg = abs(szog_tavolsag) * ero #a sebesség legyen a szögtávolság*erővel pl 1 fok akkor 1*4=4
        if sebesseg > max_sebesseg: #ha sebesség nagyobb mint max sebesség 
            sebesseg = max_sebesseg #sebesség legyen max sebesség
        if sebesseg < min_sebesseg: #ha sebesség kisebb mint min sebesség
            sebesseg = min_sebesseg #legyen sebesség min sebesség
        if szog_tavolsag > 0: #ha pozitiv a szög távolság, jobbra
            kerek_motorB.run(-sebesseg) #jobbra, -sebesség
            kerek_motorJ.run(sebesseg) #jobbra, sebesség
        else: #ha negativ a szög távolság, balra
            kerek_motorB.run(sebesseg) #balra, sebesség
            kerek_motorJ.run(-sebesseg) #balra, -sebesség

def futas_0():
    while True:
        db.straight(100)
        db.straight(-100)
        #ezt átírom
 
def futas_1():
    while True:
        db.turn(100)
 
def futas_2():
    while True:
        hub.light.on(Color.ORANGE)
        wait(500)
 
def futas_3():
    while True:
        hub.speaker.beep()
        wait(500)
        #én meg ezt írom át
 
futas = 0
futasok = [futas_0, futas_1, futas_2, futas_3]
 
while True:
    hub.display.number(futas + 1)
    megnyomva= []
 
    while not any(megnyomva):
        megnyomva = hub.buttons.pressed()
       
    while hub.buttons.pressed():
        pass
   
    if Button.CENTER in megnyomva:
        futasok[futas]()
        futas = (futas + 1) % len(futasok)
 
    elif Button.RIGHT in megnyomva:
        futas = (futas + 1) % len(futasok)
 
    elif Button.LEFT in megnyomva:
        futas = (futas - 1) % len(futasok)
 
    if Button.BLUETOOTH in megnyomva:
        break
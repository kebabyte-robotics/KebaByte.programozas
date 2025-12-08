from pybricks.hubs import *
from pybricks.pupdevices import *
from pybricks.parameters import *
from pybricks.robotics import *
from pybricks.tools import *
 
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
 
def futas_1():
    while True:
        db.straight(100)
        db.straight(-100)
 
def futas_2():
    while True:
        db.turn(100)
 
def futas_3():
    while True:
        hub.light.on(Color.ORANGE)
        wait(500)
 
def futas_4():
    while True:
        hub.speaker.beep()
        wait(500)
 
futas= 1
futasok = [futas_1, futas_2, futas_3, futas_4]
 
while True:
    hub.display.off()
    hub.display.number(futas)
    megnyomva= []
 
    while not any(megnyomva):
        megnyomva = hub.buttons.pressed()
       
    while hub.buttons.pressed():
        pass
   
    if Button.CENTER in megnyomva:
        futasok[futas - 1]()
        futas = futas % 4 + 1
 
 
    elif Button.RIGHT in megnyomva:
        futas = futas % 4 + 1
 
    elif Button.LEFT in megnyomva:
        futas = (futas - 2) % 4 + 1
 
    if Button.BLUETOOTH in megnyomva:
        break
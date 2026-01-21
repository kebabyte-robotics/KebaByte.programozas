from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
 
hub = PrimeHub()
hub.system.set_stop_button(None)
hub.system.set_stop_button(Button.BLUETOOTH)
hub = PrimeHub()


hub.display.number(1)
def futas_0():
    print("futas")
    forward(1000000)
    #pass
 
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
 
futas = 0
futasok = [futas_0, futas_1, futas_2, futas_3]
 
while True:
    print("menu")
    motor_stop()
    hub.display.number(futas + 1)
    megnyomva = []
 
    while not any(megnyomva):
        megnyomva = hub.buttons.pressed()
       
    while hub.buttons.pressed():
        pass
   
    if Button.CENTER in megnyomva:
        calibrate()
        futasok[futas]()
        futas = (futas + 1) % 4
 
    elif Button.RIGHT in megnyomva:
        futas = (futas + 1) % 4
 
    elif Button.LEFT in megnyomva:
        futas = (futas - 1) % 4
 
    if Button.BLUETOOTH in megnyomva:
        break
 
 
 

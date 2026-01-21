from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
 
hub = PrimeHub()
hub.system.set_stop_button(None)
hub.system.set_stop_button(Button.BLUETOOTH)
i = 2
hub = PrimeHub()
press = []
pressd = []
hub.display.number(1)
while True:
    while not any(press):
        press = hub.buttons.pressed()
        pressd = hub.buttons.pressed()
       
        #hub.display.text((str)('a'))
   
    #hub.display.text((str)('b'))
   
    while any(press):
        #hub.display.text((str)(pressed))
        press = hub.buttons.pressed()
       
    #hub.display.text((str)(pressd))
   
   
    if Button.CENTER in pressd:
        hub.display.off()
        hub.display.number(i)
        if i == 1:
            hub.display.pixel(3, 0, 0)
        else:
            hub.display.pixel(i-2, 0, 0)
        i = i % 4 + 1
        pressd = []
        press = []
    elif Button.RIGHT in pressd:
        hub.display.off()
        hub.display.number(i)
        i = i % 4 + 1
        pressd = []
        press = []

from pybricks.hubs import * #beimportálja az agyat
from pybricks.pupdevices import * #beimportálja a pupdevicesokat
from pybricks.parameters import * #beimportálja a paramétereket
from pybricks.tools import * #beimportálja a toolsokat

hub = PrimeHub() #az agyat elnevezi hubnak
bal  = Motor(Port.B) #a motort ami a B portba van elnevezzük balnak és óra járásával megegyező irányba forog
jobb = Motor(Port.F, Direction.COUNTERCLOCKWISE) #a motort ami az F portba van elnevezzük balnak és óra járásával ellentétes irányba forog
feltet_bal = Motor(Port.E, gears=[12, 12, 20]) #a feltét motort ami az E portba van elnevezzük feltet_balnak és a fogaskerék áttét 12, 12, 20
feltet_jobb = Motor(Port.A, gears=[12, 12, 20])  #a feltét motort ami az A portba van elnevezzük feltet_jobbnak és a fogaskerék áttét 12, 12, 20

hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
irany = 0 #létrehozunk egy irany nevű változót aminek 0 értéket adunk
feltet_bal.control.limits(1000, 5000)
feltet_jobb.control.limits(1000, 5000)

while not hub.imu.ready(): hub.display.char("x")

def egyenes(e_tavolsag, e_legkisebb_sebesseg=40, e_gyorsitas=40, e_korekcio=0.01, e_legnagyobb_sebesseg = 700, e_lassitas=80): #létrehozunk egy egyenes nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, az e_tavolsagot, e_lassitast és a e_gyorsitast mm-be adjuk meg, az alap értékek csak átlagban működnek 
    global irany #engedélyezzük a függvénynek az irany változó használatát a függvényen belül
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    e_tavolsag = e_tavolsag / 0.489 #a mm-ben megadott e_tavolsagot átváltjuk motor fokokra, a kerék kerületét elosztjuk 360nal
    e_gyorsitas /= 0.489 #a mm-ben megadott e_gyorsitast átváltjuk motor fokokra, a kerék kerületét elosztjuk 360nal
    e_lassitas /= 0.489 #a mm-ben megadott e_lassitast átváltjuk motor fokokra, a kerék kerületét elosztjuk 360nal
    while True: #elindítunk egy ciklust ami addig fut ameddig le nem állítjuk
        e_megtett_tavolsag = (bal.angle()+jobb.angle()) / 2 #a bal és jobb szöget összeadjuk és elosztjuk 2-vel, kijön az átlagos megtetttávolság ami az e_megtett_tavolsag
        e_hatralevo_tavolsag = e_tavolsag - e_megtett_tavolsag #a tavolsagból kivonjuk az e_megtett_tavolsagot így kijön a e_hatralevo_tavolsag
        e_jelzo = e_hatralevo_tavolsag/abs(e_hatralevo_tavolsag) #az e_hatralevo_tavolsagot elosztjuk az e_hatralevo_tavolsag abszolutértékével, ha az e_hatralevo_tavolsag pozitiv akkor 1, ha negatív akkor -1 lesz a e_jelző értéke(pl 10/10=1 -10/10=-1)
        e_megtett_tavolsag = abs(e_megtett_tavolsag) #az e_megtett_tavolsag abszolut értéke legyen az e_megtett_tavolsag, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv
        e_hatralevo_tavolsag = abs(e_hatralevo_tavolsag) #az e_hatralevo_tavolsag abszolut értéke legyen az e_hatralevo_tavolsag, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv
        if e_hatralevo_tavolsag < 3: #ha e_hatralevo_tavolsag kevesebb, mint 3 motorfok 
            break #akkor álj, vagyis lépjen ki a while-ból
        if e_hatralevo_tavolsag < e_lassitas : #ha az e_hatralevo_tavolsag kevesebb, mint az e_lassitas(ha már a lassításba van)
            e_ratio = e_hatralevo_tavolsag / e_lassitas #akkor a lassitas ratioja az e_hatralevo_tavolsag/e_lassitas, ez legyen az e_ratio ez a szám 0-1ig van
            e_mostani_sebesseg = max(e_ratio * e_legnagyobb_sebesseg, e_legkisebb_sebesseg) * e_jelzo #az e_mostani_sebesseg legyen a max(vagyis a nagyobb) az e_legkisebb_sebesseg(hogy ne menjen lassabban) vagy a e_ratio*e_legnagyobb_sebesseg és ez megszroozva az e_jelzo-vel ami 1 vagy -1, hogy pozitiv vagy negativ legyen
        elif e_megtett_tavolsag < e_gyorsitas: #ha nem az előző, ha az e_megtett_tavolsag kisebb, mint az e_gyorsitas(a gyorsítás szakaszba van)
            e_ratio = e_megtett_tavolsag / e_gyorsitas #az e_ratio legyen egyenlő az e_megtett_tavolsag / e_gyorsittassal ez a szám 0-1 lehet
            e_mostani_sebesseg = max(e_ratio * e_legnagyobb_sebesseg, e_legkisebb_sebesseg) * e_jelzo ##az e_mostani_sebesseg legyen a max(vagyis a nagyobb) az e_legkisebb_sebesseg(hogy ne menjen lassabban) vagy a e_ratio*e_legnagyobb_sebesseg és ez megszroozva az e_jelzo-vel ami 1 vagy -1, hogy pozitiv vagy negativ legyen
        else: #ha az előzők közül egyik sem(nem gyorsít sem lassít)
            e_mostani_sebesseg = e_legnagyobb_sebesseg * e_jelzo #az e_mostani_sebesseg legyen egyenlő az e_legnagyobb_sebesseg * e_jelzo, ami 1 vagy -1 lehet, ami pozitivvá/negatívvá alakítja a számot
        e_iranyelteres = (irany - hub.imu.heading()) * e_korekcio #az e_iranyelteres legyen egyenlő az irany(merre kellene mennie) - a tényleges mostani fok * az e_korekcio-val(azért kell korekcio hogy finoman korigálja, nem egyben nagyot)
        e_korekciomertek = e_mostani_sebesseg * e_iranyelteres * e_jelzo #az e_korekcio mertek legyen egyenlő az e_mostani_sebesseg * e_iranyelteres * e_jelzo(1,-1) ez lesz a mérték amennyivel korigálni kell
        bal.run (e_mostani_sebesseg - e_korekciomertek) #a bal az e_mostani_sebesseg - e_korekciomertek(azért -, mert ha jobbra tér akkor + eltérés akkor abból - lesz, vagyis lassít, ha balra tér el akkor - és -- az + vagyis gyorsít)
        jobb.run(e_mostani_sebesseg + e_korekciomertek) #a jobb az e_mostani_sebesseg + e_korekciomertek (azért +, mert ha jobbra tér el akkor + eltérés abból + marad, vagyis gyorsít, ha balra tér el akkor - és az - marad vagyis lassít)
    bal.hold() #a bal megáll(pont ott marad a pozíciója)
    jobb.hold()#a jobb megáll(pont ott marad a pozíciója)



def kanyarodas(k_fok, k_legnagyobb_sebesseg=360, k_lassitas=80, k_legkisebb_sebesseg=50): #létrehozunk egy kanyarodas nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, a k_fok-ot(fokban), k_legnagyobb_sebesseg, k_lassitas(fok-ban), k_legkisebb_sebesseg, az alapból beírt paramétrek csak átlagban működnek
    k_alap_fok = hub.imu.heading() #a k_alap_fok értéke legyen egyenlő a gyro értékével, azért kell,h ogy tudjon az előző tévedések alapján korigálni
    global irany #engedélyezzük a függvénynek az irany változó használatát a függvényen belül
    k_cel_fok = irany+k_fok #a k_cel_fok = az irány(amerre kellene menni)+k_fok (az irány azért kell hogy az előző tévedést kijavítja)
    irany = k_cel_fok #az irany legyen egyenlő a k_cel_fok
    k_cel_fok -= k_alap_fok #a k_cel_fok-ból vonjuk ki a k_alap_fokot vagyi a tévedést
    while True: #addig fut a ciklus amíg ki nem lépünk
        k_megtett_fokok = hub.imu.heading() - k_alap_fok #a k_megtett_fokok legyen egyenlő a mostani gyro - a k_alap_fok
        k_hatralevo_fokok = k_cel_fok - k_megtett_fokok #a k_hatralevo_fokok legyen egyenlő  a k_cel_fok - k_megtett_fokok
        k_jelzo = k_hatralevo_fokok/abs(k_hatralevo_fokok) #a k_hatralevo_fokokat elosztjuk a k_hatralevo_fokok abszolutértékével, ha a k_hatralevo_fokok pozitiv akkor 1, ha negatív akkor -1 lesz a K_jelző értéke(pl 10/10=1 -10/10=-1)
        k_hatralevo_fokok = abs(k_hatralevo_fokok) #a k_hatralevo_fokok abszolut értéke legyen a k_hatralevo_fokok, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv
        if k_hatralevo_fokok <= 0.5: #ha a k_hatralevo_fokok kisebb egyenlő mint 0.5
            break #akkor álj, vagyis lépjen ki a while-ból       
        if k_hatralevo_fokok < k_lassitas : #ha a k_hatralevo_fokok kisebb, mint a k_lassitas(vagyis a lassítás szakaszban van)
            k_ratio = k_hatralevo_fokok / k_lassitas #akkor a lassitas ratioja a k_hatralevo_fokok/k_lassitas, ez legyen az e_ratio ez a szám 0-1ig van 
            k_mostani_sebesseg = max(k_ratio * k_legnagyobb_sebesseg, k_legkisebb_sebesseg) * k_jelzo #az k_mostani_sebesseg legyen a max(vagyis a nagyobb) az k_legkisebb_sebesseg(hogy ne menjen lassabban) vagy a k_ratio*k_legnagyobb_sebesseg és ez megszorozva az k_jelzo-vel ami 1 vagy -1, hogy pozitiv vagy negativ legyen
        else: #ha nem az előző, vagyis nem lassít
            k_mostani_sebesseg = k_legnagyobb_sebesseg * k_jelzo #akkor a k_mostani_sebesseg legyen egyenlő a k_legnagyobb_sebesseg * k_jelzo(1 vagy -1)
        bal.run(-k_mostani_sebesseg) #a bal a -k_mostani_sebesseg (azért -, mert ha jobbra forog akkor + eltérés akkor abból - lesz, vagyis hatra megy, ha balra tér el akkor - és -- az + vagyis elore megy)
        jobb.run(k_mostani_sebesseg) #a jobb a k_mostani_sebesseg (azért +, mert ha jobbra forog akkor + eltérés abból + marad, vagyis elore megy, ha balra tér el akkor - és az - marad vagyis hatra megy)
    bal.hold() #a bal megáll(pont ott marad a pozíciója)
    jobb.hold() #a jobb megáll(pont ott marad a pozíciója)
    wait(100)

def jobb_feltet(angle, speed = 400): #létrehozunk egy jobb_feltet nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a jobb feltét motorra vonatkozik
    feltet_jobb.run_angle(speed, angle)

def bal_feltet(angle, speed = 400): #létrehozunk egy bal_feltet nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a bal feltét motorra vonatkozik
    feltet_bal.run_angle(speed, angle)

def jobb_feltet_hatter(angle, speed = 400): #létrehozunk egy jobb_feltet_hatter nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a jobb feltét motorra vonatkozik és ez mozgás közben is működik
    feltet_jobb.run_angle(speed, angle, wait=False)

def bal_feltet_hatter(angle, speed = 400): #létrehozunk egy bal_feltet_hatter nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a bal feltét motorra vonatkozik és ez mozgás közben
    feltet_bal.run_angle(speed, angle, wait=False)

voltage = hub.battery.voltage()
print(voltage)
hub.system.set_stop_button(Button.BLUETOOTH) #a leállító gombot berakjuk a bluetooth gombra
hub.display.number(1) #kiirja az 1-es számot, mert az elején az 1-es futás van

def futas_1(): #létrehozunk egy futas_1 nevü függvényt
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    wait(200)
    #ez után ird
    jobb_feltet_hatter (190)
    egyenes (650, e_legkisebb_sebesseg=650)
    egyenes (-164)
    kanyarodas (40)
    egyenes (250)
    kanyarodas (-86)
    egyenes (56, 20)
    jobb_feltet_hatter (-10)
    egyenes (57)
    jobb_feltet (-158)
    egyenes (60)
    kanyarodas (9)
    jobb_feltet (50)
    egyenes (-30)
    jobb_feltet (450)
    egyenes (-70)
    jobb_feltet_hatter (100)
    bal_feltet (-180)
    kanyarodas (47, k_legkisebb_sebesseg=300)
    egyenes (-1000, e_legkisebb_sebesseg=800, e_legnagyobb_sebesseg=900)
 
def futas_2(): #létrehozunk egy futas_2 nevü függvényt
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    wait(200)
    #ez után irdjobb_feltet_hatter (-240)
    jobb_feltet (300)
    jobb_feltet_hatter (-240)
    egyenes (960)
    kanyarodas (90)
    jobb_feltet (240)
    egyenes (110)
    jobb_feltet (-200, 100)
    jobb_feltet_hatter(1)
    egyenes (-20)
    jobb_feltet_hatter (300, speed=300)
    kanyarodas (46)
    egyenes (280)
    jobb_feltet (-170, speed=321)
    jobb_feltet_hatter(1)
    egyenes (-40)
    kanyarodas (-55)
    jobb_feltet_hatter (65)
    egyenes (300)
    kanyarodas (35)
    egyenes (150)
    jobb_feltet_hatter (70)
    kanyarodas (-25)
    jobb_feltet_hatter(-150)
    egyenes(-15)
    #egyenes (450)
    #kanyarodas (60)
    #egyenes (700)
 
 
def futas_3(): #létrehozunk egy futas_3 nevü függvényt
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    wait(200)
    #ez után ird
    egyenes(61)
    kanyarodas(25)
    egyenes(339)
    kanyarodas(-25)
    egyenes(321, e_legkisebb_sebesseg=700, e_legnagyobb_sebesseg=1000)
    kanyarodas(-34)
    egyenes(29)
    kanyarodas(55)
    egyenes(41)
 
def futas_4(): #létrehozunk egy futas_4 nevü függvényt
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    wait(200)
    #ez után ird
    egyenes(370, e_korekcio=0.007)
    jobb_feltet(180, 999)
    jobb_feltet(-180, 999)
    wait(100)
    jobb_feltet(230, 999)
    jobb_feltet(-270, 999)
    wait(100)
    jobb_feltet(230, 999)
    jobb_feltet(-270, 999)
    wait(100)
    jobb_feltet(240, 999)
    jobb_feltet(-320, 999)
    kanyarodas(-32)
    egyenes(225)
    kanyarodas(64, k_legnagyobb_sebesseg=400)
    egyenes(27)
    kanyarodas(-37.6)
    egyenes(32)
    kanyarodas(-15, k_legkisebb_sebesseg=150, k_lassitas=16)
    egyenes(-35)
    kanyarodas(23)
    egyenes(-159, e_korekcio=0.005)
    kanyarodas(-136.5)
    egyenes(330)
    kanyarodas(66.5)
    jobb_feltet(250)
    kanyarodas(-168)
    egyenes(360)

def futas_5(): #létrehozunk egy futas_5 nevü függvény
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    wait(200)
    #ez után ird
    egyenes(230)
    kanyarodas(35)
    egyenes(495)
    kanyarodas(-79)
    egyenes(242, e_legnagyobb_sebesseg=950)
    egyenes(-200)
    kanyarodas(126)
    egyenes(-300)
    egyenes(61)
    kanyarodas(-45)
    egyenes(-21)
    kanyarodas(-56)
    egyenes(10)
    kanyarodas(-3)
    egyenes(-21)
    bal_feltet(2200)
    egyenes(11)
    kanyarodas(65)
    """ egyenes(230)
    kanyarodas(35)
    egyenes(205)
    kanyarodas(-35)
    egyenes(270) """
 
futas = 0 #létrehozzunk egy futas változót aminek 0 az értéke, ami számolja hányadik futás
futasok = [futas_1, futas_2, futas_3, futas_4, futas_5] #létrehozunk egy futasok nevű tömböt és megadjuk az elemeit
max_futas = len(futasok) #a max_futast létre hozzuk és az értéke a futasok elemszáma

while True: #egy ciklus ami addig fut amig nem lépünk ki
    hub.display.number(futas + 1) #az agy irja ki a futast + 1
    megnyomva = [] #létrehozunk egy tömböt aminek megnyomva a neve és üresre állítjuk
    while not any(megnyomva): #amig nincs semmi a megnyomva tömbben, addig fusson
        megnyomva = hub.buttons.pressed() #a megnyomva tömbön legyen az agyon megnyomott gombok  
    lenyomott = StopWatch() #létrehozzuk a lenyomott változót, amin elindul a stopper
    rezgett = False #létrehozzunk egy rezgett nevű változót amit beállítunk Hamisra
    while hub.buttons.pressed(): #egy ciklus ami addig fut ameddig egy gomb le van nyomva
        if lenyomott.time() > 500: #ha a lenyomott idő több mint fél másodperc
            rezgett = True #a rezgett legyen Igaz
            bal_feltet_hatter(45, speed=900) #bal_feltet_hattert elindítjuk 45 fokra, 600 sebességgel
            jobb_feltet(45, speed=900) #jobb_feltet elindítjuk 45 fokra, 600 sebességgel
            bal_feltet_hatter(-45, speed=900) #bal_feltet_hattert elindítjuk 45 fokra, 600 sebességgel
            jobb_feltet(-45, speed=900) #jobb_feltet elindítjuk 45 fokra, 600 sebességgel
        pass #menjen tovább
    if rezgett: #ha rezgett már
        continue  #menjen tovább

    if Button.RIGHT in megnyomva: #ha a jobb gomb bent van a megnyomvában
        futas = (futas + 1) % max_futas #a futas egyen egyenlő a futas + 1nak a maradéka a max_futassal
    if Button.LEFT in megnyomva: #ha a bal gomb bent van a megnyomvában
        futas = (futas - 1) % max_futas #a futas egyen egyenlő a futas + 1nak a maradéka a max_futassal
    if Button.CENTER in megnyomva: #ha a középső gomb bent van a megnyomvában
        irany = 0
        hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk

        try: #futassd le a kódot, ha kilép menj az exceptre
            hub.system.set_stop_button(Button.CENTER) # a stop button legyen a középső gomb
            futasok[futas]() #futassa le a futasok futas elemét
            futas = (futas + 1) % max_futas #a futas egyen egyenlő a futas + 1nak a maradéka a max_futassal
        except SystemExit: #ha ki akar lépni ezt csináld
            bal.hold() #bal áljon le
            jobb.hold() #jobb áljon le
            feltet_bal.hold() #feltet_bal álljon le
            feltet_jobb.hold() #feltet_jobb álljon le
            while Button.CENTER in hub.buttons.pressed(): #a középső gomb megvan nyomva addig
                pass #menjen tovább
        hub.system.set_stop_button(Button.BLUETOOTH) # a stop button legyen a bluetooth

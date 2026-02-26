from pybricks.hubs import * #beimportálja az agyat
from pybricks.pupdevices import * #beimportálja a pupdevicesokat
from pybricks.parameters import * #beimportálja a paramétereket
from pybricks.tools import * #beimportálja a toolsokat
 
hub = PrimeHub() #az agyat elnevezi hubnak
bal  = Motor(Port.B) #a motort ami a B portba van elnevezzük balnak és óra járásával megegyező irányba forog
jobb = Motor(Port.F, Direction.COUNTERCLOCKWISE) #a motort ami az F portba van elnevezzük balnak és óra járásával ellentétes irányba forog
feltet_bal = Motor(Port.E) #a feltét motort ami az E portba van elnevezzük feltet_balnak és a fogaskerék áttét 12, 12, 20
feltet_jobb = Motor(Port.A)  #a feltét motort ami az A portba van elnevezzük feltet_jobbnak és a fogaskerék áttét 12, 12, 20
feltet_bal.control.limits(1000, 6500)
feltet_jobb.control.limits(1000, 6500)
bal.control.limits(2000, 5500)
jobb.control.limits(2000, 5500)
while not hub.imu.ready(): hub.display.char("x")
 
hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
irany = 0 #létrehozunk egy irany nevű változót aminek 0 értéket adunk
jobb_attet = 1 #létrehozunk egy jobb_attet nevű változót aminek 1 értéket adunk, ez fogja számolni a fogaskerék áttéteket
bal_attet = 1 #létrehozunk egy jobb_attet nevű változót aminek 1 értéket adunk, ez fogja számolni a fogaskerék áttéteket
 
 
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
    feltet_jobb.run_angle(speed, angle*jobb_attet) #a feltet_jobbot lefutatjuk fokban, itt paraméterként a speed és az angle van megadva beszorozva a jobb_atettel
 
def bal_feltet(angle, speed = 400): #létrehozunk egy bal_feltet nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a bal feltét motorra vonatkozik
    feltet_bal.run_angle(speed, angle*bal_attet) #a feltet_balt lefutatjuk fokban, itt paraméterként a speed és az angle van megadva beszorozva a bal_atettel
 
def jobb_feltet_hatter(angle, speed = 400): #létrehozunk egy jobb_feltet_hatter nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a jobb feltét motorra vonatkozik és ez mozgás közben is működik
    feltet_jobb.run_angle(speed, angle*jobb_attet, wait=False) #a feltet_jobbat lefutatjuk fokban, itt paraméterként a speed és az angle van megadva beszorozva a jobb_atettel és itt a nem vár hanem lefut hatterben
 
def bal_feltet_hatter(angle, speed = 400): #létrehozunk egy bal_feltet_hatter nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a bal feltét motorra vonatkozik és ez mozgás közben
    feltet_bal.run_angle(speed, angle*bal_attet, wait=False) #a feltet_balt lefutatjuk fokban, itt paraméterként a speed és az angle van megadva beszorozva a bal_atettel és itt a nem vár hanem lefut hatterben
 
hub.system.set_stop_button(Button.BLUETOOTH) #a leállító gombot berakjuk a bluetooth gombra
hub.display.number(1) #kiirja az 1-es számot, mert az elején az 1-es futás van
voltage = hub.battery.voltage()
print(voltage)
 
def futas_1(): #létrehozunk egy futas_1 nevü függvényt
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    wait(200)
    #ez után ird
    """bal_feltet (-115, speed=100) #felemelei a bal feltétkart
    jobb_feltet_hatter(65) #felemeli a jobb feltétkart
    egyenes (635) #előre megy az 1-es feladathoz
    bal_feltet (79, 100) #leereszti az ecsetre a bal feltétkart
    egyenes (60) #előre megy ezzel megoldva a hátsó talajlerakódást
    egyenes (-110) #hátra megy ezzel megoldva az elöl lévő talajlerakódást
    egyenes (50) #előre megy
    bal_feltet (-59, 100) #felemeli a bal feltétet az ecsettel együtt
    egyenes (70) # belemegy a passzívkeretbe
    kanyarodas (87) #elkanyarodik jobbra
    egyenes(72) #előre megy, hogy megfelelő pozíciót szerezzen a 2. feladathoz
    kanyarodas(-128) #rákanyarodik a 2-es feladatra
    egyenes(330) #rá megy a 2-es feladatra és a passzívkerettel megoldva a 2/3 részét a feladatnak és jobb oldali feltét meg az utolsó fedőtalaj harmadra megy rá
    jobb_feltet(213, speed=100) #felemeli a jobb oldali feltétet ezzel megoldva az utolsó harmadát a kettes feladatnak
    egyenes(-277) #hátra megy, ezzel eltávolodva a passzívkeretből és a feladattól
    kanyarodas(115.5) #rákanyarodik
    egyenes(-76)
    jobb_feltet(-265)
    egyenes(72.72)
    jobb_feltet(245, speed=70)
    egyenes(-235)
    kanyarodas(119)
    egyenes(900)"""
    bal_feltet (-115, speed=100)
    jobb_feltet_hatter(50)
    egyenes (635)
    bal_feltet (65, 100)
    egyenes (60)
    egyenes (-110)
    egyenes (50)
    bal_feltet (-53, 120)
    egyenes (65)
    kanyarodas (88)
    egyenes(100)
    kanyarodas(-125.1)
    egyenes(330)
    jobb_feltet(210)
    wait(100)
    egyenes(-300)
    kanyarodas(93)
    egyenes(-89)
    jobb_feltet(-257)
    egyenes(99.99)
    jobb_feltet(235, speed=70)
    egyenes(-245)
    kanyarodas(133)
    egyenes(850)
 
 
def futas_2(): #létrehozunk egy futas_2 nevü függvényt
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    wait(200)
    #ez után irdjobb_feltet_hatter (-240)
    egyenes(666, e_legnagyobb_sebesseg=800)
    jobb_feltet(255, speed=200)
    bal_feltet(2200, speed=888)
    egyenes(-15)
    jobb_feltet(-230)
    egyenes(-777, e_legnagyobb_sebesseg=999)
 
def futas_3(): #létrehozunk egy futas_3 nevü függvényt
    global jobb_attet, bal_attet #engedélyezzük a függvénynek a jobb_attet és a bal_attet változó használatát a függvényen belül
    jobb_attet = 1
    bal_attet = 1
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    wait(200)
    #ez után ird
    bal_feltet_hatter(-400)
    egyenes(120)
    kanyarodas(56.3)
    egyenes(825)
    jobb_feltet(-230, speed=600)
    kanyarodas(-35)
    jobb_feltet_hatter(190)
    egyenes(370)
    egyenes(-84)
    jobb_feltet_hatter(-235)
    kanyarodas(99)
    egyenes(189)
    egyenes(-129)
    jobb_feltet(245)
    kanyarodas(-46.4)
    egyenes(235)
    kanyarodas(17)
    egyenes(-45)
    bal_feltet(395)
    egyenes(35)
    kanyarodas(11.5)
    egyenes(30)
    egyenes(-25)
    kanyarodas(-11.5)
    egyenes(-115)
    kanyarodas(9)
    bal_feltet(-355)
    kanyarodas(19)
    egyenes(300)
    kanyarodas(-69.2)
    egyenes(125)
    bal_feltet(355, speed=550)
    bal_feltet(-350)
    egyenes(-20)
    kanyarodas(72.69)
    egyenes(55)
    jobb_feltet(-210, speed=899)
    jobb_feltet(230, speed=799)
    jobb_feltet(-230, speed=799)
    jobb_feltet(230, speed=799)
    jobb_feltet(-230, speed=899)
    jobb_feltet(240, speed=799)
    kanyarodas(53)
    egyenes(690, e_legkisebb_sebesseg=400, e_legnagyobb_sebesseg=999)
   
 
def futas_4(): #létrehozunk egy futas_4 nevü függvényt
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    wait(200)
    #ez után ird
    jobb_feltet_hatter(500)
    egyenes(120)
    kanyarodas(-49.5)
    jobb_feltet_hatter(-500, speed=999)
    egyenes(243)
    kanyarodas(-23, k_legnagyobb_sebesseg=720, k_legkisebb_sebesseg=360)
    egyenes(-30)
    kanyarodas(-11)
    jobb_feltet(350, speed=999)
    egyenes(200)
    kanyarodas(6)
    jobb_feltet(-360, speed=995)
    wait(100)
    kanyarodas(-25, k_legkisebb_sebesseg=180)
    egyenes(360)
    kanyarodas(53.3, k_legkisebb_sebesseg=180)
    egyenes(172)
    jobb_feltet(210, speed=400)
    kanyarodas(18)
    egyenes(24)
    bal_feltet(214)
    egyenes(-211)
 
futas = 0 #létrehozzunk egy futas változót aminek 0 az értéke, ami számolja hányadik futás
futasok = [futas_1, futas_2, futas_3, futas_4] #létrehozunk egy futasok nevű tömböt és megadjuk az elemeit
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
            while Button.CENTER in hub.buttons.pressed(): #a középső gomb megvan nyomva addig
                pass #menjen tovább
        bal.stop() #bal áljon le
        jobb.stop() #jobb áljon le
        feltet_bal.stop() #feltet_bal álljon le
        feltet_jobb.stop() #feltet_jobb álljon le
        hub.system.set_stop_button(Button.BLUETOOTH) # a stop button legyen a bluetooth





















































































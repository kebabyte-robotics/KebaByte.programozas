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

def jobb_feltet(angle, speed = 400): #létrehozunk egy jobb_feltet nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a jobb feltét motorra vonatkozik
    feltet_jobb.run_angle(speed, angle)

def bal_feltet(angle, speed = 400): #létrehozunk egy bal_feltet nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a bal feltét motorra vonatkozik
    feltet_bal.run_angle(speed, angle)

def jobb_feltet_hatter(angle, speed = 400): #létrehozunk egy jobb_feltet_hatter nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a jobb feltét motorra vonatkozik és ez mozgás közben is működik
    feltet_jobb.run_angle(speed, angle, wait=False)

def bal_feltet_hatter(angle, speed = 400): #létrehozunk egy bal_feltet_hatter nevű függvényt, amiben paraméterként benne van a fok és a sebesség és ez a bal feltét motorra vonatkozik és ez mozgás közben
    feltet_bal.run_angle(speed, angle, wait=False)

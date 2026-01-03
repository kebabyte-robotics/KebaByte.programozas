from pybricks.hubs import * #beimportálja az agyat
from pybricks.pupdevices import * #beimportálja a pupdevicesokat
from pybricks.parameters import * #beimportálja a paramétereket
from pybricks.tools import * #beimportálja a toolsokat


hub = PrimeHub() #az agyat elnevezi hubnak
bal  = Motor(Port.B) #a motort ami a B portba van elnevezzük balnak és óra járásával megegyező irányba forog
jobb = Motor(Port.F, Direction.COUNTERCLOCKWISE) #a motort ami az F portba van elnevezzük balnak és óra járásával ellentétes irányba forog
 
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
            break #akkor álj
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



def kanyarodas(k_fok, k_legnagyobb_sebesseg=360, k_lassitas=80, k_legkisebb_sebesseg=50):
 
    k_alap_fok = hub.imu.heading()
    global irany
    k_cel_fok = irany+k_fok
    irany = k_cel_fok
    k_cel_fok -= k_alap_fok
    while True:
        k_megtett_fokok = hub.imu.heading() - k_alap_fok
        k_tovabbi_fokok = k_cel_fok - k_megtett_fokok
        k_jelzo = k_tovabbi_fokok/abs(k_tovabbi_fokok)
        k_tovabbi_fokok = abs(k_tovabbi_fokok)
        if k_tovabbi_fokok <= 0.5:
            break
        
        if k_tovabbi_fokok < k_lassitas :
            k_ratio = k_tovabbi_fokok / k_lassitas
            k_mostani_sebesseg = max(k_ratio * k_legnagyobb_sebesseg, k_legkisebb_sebesseg) * k_jelzo
        else:
            k_mostani_sebesseg = k_legnagyobb_sebesseg * k_jelzo
        
        bal.run(-k_mostani_sebesseg)
        jobb.run(k_mostani_sebesseg)
    bal.hold()
    jobb.hold()
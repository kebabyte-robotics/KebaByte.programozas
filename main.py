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
        e_megtett_tavolsag = (bal.angle()+jobb.angle()) / 2
        e_hatralevo_tavolsag = e_tavolsag - e_megtett_tavolsag
        e_jelzo = e_hatralevo_tavolsag/abs(e_hatralevo_tavolsag)
        e_megtett_tavolsag = abs(e_megtett_tavolsag)
        e_hatralevo_tavolsag = abs(e_hatralevo_tavolsag)
        if abs(e_hatralevo_tavolsag) < 3:
            break
        if e_hatralevo_tavolsag < e_lassitas :
            e_ratio = e_hatralevo_tavolsag / e_lassitas
            e_mostani_sebesseg = max(e_ratio * e_legnagyobb_sebesseg, e_legkisebb_sebesseg) * e_jelzo
        elif e_megtett_tavolsag < e_gyorsitas:
            e_ratio = e_megtett_tavolsag / e_gyorsitas
            e_mostani_sebesseg = max(e_ratio * e_legnagyobb_sebesseg, e_legkisebb_sebesseg) * e_jelzo
        else:
            e_mostani_sebesseg = e_legnagyobb_sebesseg * e_jelzo
        e_iranyelteres = (irany - hub.imu.heading()) * e_korekcio
        e_korekciomertek = e_mostani_sebesseg * e_iranyelteres * e_jelzo
        bal.run (e_mostani_sebesseg - e_korekciomertek)
        jobb.run(e_mostani_sebesseg + e_korekciomertek)
    bal.hold()
    jobb.hold()



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
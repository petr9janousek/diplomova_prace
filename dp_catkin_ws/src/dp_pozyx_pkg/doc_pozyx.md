# Informace k použití Pozyx UWB

**Odkazy**

* [články Pozyx o UWB](https://www.pozyx.io/technology/pozyx-academy)
* [články Pozyx tutorialy](https://docs.pozyx.io/creator/)
* [python Pozyx API](https://readthedocs.org/projects/pypozyx/)
* [python Pozyx github](https://github.com/pozyxLabs/Pozyx-Python-library)
* [příspěvek na blogu v cz jazyce](https://robotika.cz/articles/pozyx/cs)
* [examples Pozyx ROS](https://github.com/pozyxLabs/pozyx_ros)

Chybějící články v dokumentaci patrně během posledního roku přibyly doporučuji projít zejména tutoriály.

### Souprava, rozmístění, zařízení...

Pozyx creator kit obsahuje tzv. "anchors" = kotvy a "tags" = štítky. Anchors by měly být umístěny staticky v prostředí, robot je má umístěny přímo na sobě (nižší přesnost vykoupená neomezenou mobilitou). Tags by měly být umístěny co nejdále od kovových částí, jedna je umístěna v robotu, jedna v přívesku pro uživatele. Dále obsahuje powerbanku a arduino klon. ID zařízení:

* 0x6a30 - kotva
* 0x6a48 - kotva
* 0x6a.. - kotva
* 0x6a.. - tag (robot)
* 0x6a.. - tag (přenosný)

V zařízeních je aktualizován firmware na verzi 1.1.

## Nastavení pozyx
* [Krátký popis parametrů](https://docs.pozyx.io/creator/configuration-of-the-uwb-parameters-arduino)
* [Performance pomocí různých parametrů](https://www.pozyx.io/creator-system-performance)
* [Detailní popis registrů, datasheet](https://docs.pozyx.io/creator/configuration-of-the-uwb-parameters-arduino)

Klíčové je nastavení kotev k dosžení dostatečné měřící frekvence. Zdá se že *creator kit* je omezen na 60Hz pravděpodobně z business důvodů, než technických.

Je monžné snížit počet kotev - což vede na nižší přesnost. Nebo změnit bitrate nebo délka preambule.

    #higher bitrate and lower PLEN 512       
    settings = UWBSettings(5,1,2,0x34,11.5) 

## ROS package

Package pro ROS vychází z oficiálních examplů a pypozyx (odkazy výše). V python je možné je importovat pomocí např: `from dp_pozyx_pkg.msg import EulerAngles`

* Custom messages - balík definuje 2 nové zprávy  
    * EulerAngles - float yaw, float pitch, float roll
    * DeviceRange - uint32 timestamp, uint32 distance, int16 RSS

### dp_uwb_utils.py
Obsahuje nástroje jako připojení zařízení, vyčtení chybové hlášky, kontrola a změna nastavení zařízení, skenování sítě a výpis různých parametrů. 

Samostatně je spouštěn pouze při debuggování, je zamýšlen jako soubor funkcí k jednotlivému importování.

#### dp_uwb_alg1.py
Obsahuje třídu Positioning_Alg1, která měří data, využívá algoritmus POSITIONING_ALGORITHM_UWB_ONLY. Nastavení je možné změnit v úvodu dokumentu, pokud má správně pracovat, je nezbytné mít správně nakonfigurovány kotvy, jejich ID, souřadnice a remote_id.

Script není python ROS NODE, pokud je spuštěn pomocí rosrun, pouze vypisuje získaná data (je možné použít pro debug).

#### dp_uwb_imu.py
Obsahuje třídu IMUsensor, experimentálně využívá IMU k měření současné polohy z tagu vestavěného v robotu.

Script není python ROS NODE, pokud je spuštěn pomocí rosrun, pouze vypisuje získaná data (je možné použít pro debug).


#### dp_uwb_pub_pose.py
Pro měření a odesílání polohy pomocí UWB, je v současnosti implementována třída DP_UWB_Positioning, která běží jako ROS NODE a publikuje polou jako "pose" jako point+quaternion. Data v geometry_msgs.Pose:

    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
      

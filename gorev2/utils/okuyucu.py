"""Kare + konum + yaw + irtifayi AYNI ANA ait olacak sekilde okur.

NEDEN BU DOSYA VAR
  Ilk konum testinde dort hedefte de sabit ~1.8 m hata cikti. Sebep sekil
  tespiti degildi: kamera karesi ile drone konumu farkli anlara aitti.
  recv_match(blocking=True) kuyrugun BASINDAN okur. Otopilot ~4 Hz
  gonderiyor, dongu 2 Hz okuyorsa her saniye 2 mesaj birikir ve okudugun
  konum giderek gecmise kayar.

  Cozum: kuyrugu bosaltip elde kalan EN SON mesaji kullanmak, ve olcumu
  ancak drone DURDUKTAN sonra almak.

ROLL/PITCH NEDEN OKUNUYOR
  Gimbal stabilize DEGIL (7 Eylul 2026 kosusunda olculdu: duraga varir
  varmaz alinan kareler 3-5 m sapti, son kareler dogruydu). Govde egikken
  cekilen kare kullanilmaz. 25 m'de 2 derece egim = 0.87 m hata.
"""

import time
import math
import cv2
import numpy as np
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.double_pb2 import Double

GZ_KAMERA = ("/world/iris_runway/model/iris_with_gimbal"
             "/model/gimbal/link/pitch_link/sensor/camera/image")
GZ_GIMBAL = "/gimbal/cmd_pitch"

GIMBAL_ASAGI = 1.57       # +1.57 = ASAGI. Eksi verirsen kamera yukari doner.
DURGUN_HIZ = 0.15         # m/s. Bunun altina inince "durdu" sayilir.


class Okuyucu:
    """Otopilot ve Gazebo kamerasindan es-zamanli veri toplar."""

    def __init__(self, captan):
        self.captan = captan            # mavutil baglantisi, disaridan gelir
        self.son_kare = None            # kamera geri cagriminin yazdigi yer

        self.kuzey = None
        self.dogu = None
        self.irtifa = None              # yerden yukseklik (m, pozitif)
        self.yaw = None                 # radyan, 0 = burun kuzeyde
        self.roll = None                # radyan, yana yatma
        self.pitch = None               # radyan, one/arkaya egilme
        self.hiz = None                 # yatay hiz buyuklugu (m/s)

        self.gz = Node()
        self.gz.subscribe(Image, GZ_KAMERA, self._kamera_geldi)

    # ---------- GAZEBO KAMERA ----------
    def _kamera_geldi(self, msg):
        """Gazebo her yeni kareyi buraya gonderir. Ayri is parcaciginda calisir."""
        kanal = msg.step // msg.width
        dizi = np.frombuffer(msg.data, dtype=np.uint8)
        dizi = dizi.reshape((msg.height, msg.width, kanal))
        if kanal == 3:
            # Gazebo RGB verir, OpenCV BGR bekler. Atlanirsa hata GELMEZ,
            # ama mavi/kirmizi ters olur ve tum tespit sessizce bozulur.
            dizi = cv2.cvtColor(dizi, cv2.COLOR_RGB2BGR)
        self.son_kare = dizi

    def gimbal_asagi(self):
        """Kamerayi tam asagi cevirir."""
        yayinci = self.gz.advertise(GZ_GIMBAL, Double)
        time.sleep(0.5)                 # advertise'dan sonra beklemek SART,
                                        # beklemezsen ilk mesajlar bosluga gider
        mesaj = Double()
        mesaj.data = GIMBAL_ASAGI
        for _ in range(10):
            yayinci.publish(mesaj)
            time.sleep(0.2)
        self._yayinci = yayinci         # nesne yasasin diye sakliyoruz
        print("Gimbal asagi cevrildi (%s)." % GIMBAL_ASAGI)

    # ---------- MAVLINK ----------
    def kuyrugu_bosalt(self):
        """Birikmis TUM mesajlari okur, her turden en sonuncusunu saklar.

        blocking=False -> mesaj yoksa None doner, bekleme yapmaz.
        None gelene kadar donduk mu, kuyrukta hicbir sey kalmamis demektir.
        """
        sayac = 0
        while True:
            m = self.captan.recv_match(blocking=False)
            if m is None:
                return sayac
            sayac += 1
            t = m.get_type()

            if t == "LOCAL_POSITION_NED":
                self.kuzey = m.x
                self.dogu = m.y
                self.irtifa = -m.z                   # NED'de asagi pozitif
                self.hiz = (m.vx ** 2 + m.vy ** 2) ** 0.5
            elif t == "ATTITUDE":
                self.yaw = m.yaw                     # radyan
                self.roll = m.roll
                self.pitch = m.pitch
            elif t == "STATUSTEXT":
                print("  [OTOPILOT]", m.text)

    def hazir_mi(self):
        """Konum hesabi icin gereken tum degerler geldi mi."""
        return None not in (self.kuzey, self.dogu, self.irtifa,
                            self.yaw, self.roll, self.pitch)

    def egim_derece(self):
        """Govdenin yataydan sapmasi (derece). roll ve pitch'in buyugu."""
        if self.roll is None or self.pitch is None:
            return 999.0
        return max(abs(math.degrees(self.roll)),
                   abs(math.degrees(self.pitch)))

    # ---------- ES-ZAMANLI OLCUM ----------
    def durgunlugu_bekle(self, azami_egim=1.5, azami_sn=15.0):
        """Drone hem YAVASLAYANA hem YATAYA gelene kadar bekler.

        Iki kosul birden: hiz < DURGUN_HIZ ve egim < azami_egim.
        Sadece hiza bakmak yetmiyor -- 30 m kosusunda drone yavaslamisti
        ama hala egikti, ilk kareler 3-5 m sapti.
        """
        bitis = time.time() + azami_sn
        while time.time() < bitis:
            self.kuyrugu_bosalt()
            if (self.hiz is not None and self.hiz < DURGUN_HIZ
                    and self.egim_derece() < azami_egim):
                return True
            time.sleep(0.1)
        return False

    def olcum_al(self, azami_sn=3.0):
        """Kare ve konumu ayni ana ait olacak sekilde alir.

        Sira onemli:
          1) kuyrugu bosalt        -> konum guncel
          2) yeni kare bekle       -> kare, o konumdan SONRA cekilmis
          3) kuyrugu tekrar bosalt -> konum kareye tekrar yaklastirildi
        """
        self.kuyrugu_bosalt()

        # Yeni kare gelsin diye eldekini sil, kamera 10 Hz calisiyor.
        self.son_kare = None
        bitis = time.time() + azami_sn
        while self.son_kare is None:
            if time.time() > bitis:
                print("UYARI: kare gelmedi.")
                return None
            time.sleep(0.02)

        kare = self.son_kare.copy()      # geri cagrim ustune yazmasin diye kopya
        self.kuyrugu_bosalt()

        if not self.hazir_mi():
            print("UYARI: konum/yaw/attitude verisi eksik.")
            return None

        return {
            "kare": kare,
            "kuzey": self.kuzey,
            "dogu": self.dogu,
            "irtifa": self.irtifa,
            "yaw": self.yaw,
            "roll": self.roll,
            "pitch": self.pitch,
            "hiz": self.hiz,
            "egim": self.egim_derece(),
        }

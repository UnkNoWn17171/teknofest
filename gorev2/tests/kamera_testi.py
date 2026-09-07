import cv2                                      # goruntu isleme ve pencere gosterimi
import numpy as np                              # ham baytlari 2 boyutlu diziye cevirmek icin
import time
from gz.transport13 import Node                 # Gazebo'nun mesaj sistemine baglanan sinif
from gz.msgs10.image_pb2 import Image           # kamera mesajinin yapisini tarif eden sinif

TOPIC = ("/world/iris_runway/model/iris_with_gimbal"
         "/model/gimbal/link/pitch_link/sensor/camera/image")

son_kare = None                                 # en son gelen goruntu burada tutulur
sayac = 0                                       # kac kare geldigini sayar


def kamera_geldi(msg):
    """Gazebo her yeni kare urettiginde bu fonksiyonu kendisi cagirir."""
    global son_kare, sayac

    kanal = msg.step // msg.width               # bir pikselde kac bayt var (RGB ise 3)
    dizi = np.frombuffer(msg.data, dtype=np.uint8)   # duz bayt yigini -> sayi dizisi
    dizi = dizi.reshape((msg.height, msg.width, kanal))  # duz diziyi resim seklinde katla

    if kanal == 3:
        dizi = cv2.cvtColor(dizi, cv2.COLOR_RGB2BGR)  # Gazebo RGB verir, OpenCV BGR bekler

    son_kare = dizi
    sayac += 1


node = Node()                                   # Gazebo agina baglanan dugum
basarili = node.subscribe(Image, TOPIC, kamera_geldi)   # topic'e abone ol

if not basarili:
    print("HATA: topic'e abone olunamadi. Sim acik mi? Topic adi dogru mu?")
    raise SystemExit(1)

print("Abone olundu. Goruntu bekleniyor... (pencerede q = cikis)")

baslangic = time.time()
while True:
    if son_kare is not None:
        cv2.imshow("Gazebo kamera", son_kare)   # pencerede goster
    else:
        if time.time() - baslangic > 10:
            print("10 sn'de tek kare gelmedi. Drone'un kamerasi kapali olabilir.")
            break

    if cv2.waitKey(30) & 0xFF == ord('q'):      # 30 ms bekle, q'ya basildiysa cik
        break

cv2.destroyAllWindows()
print(f"Toplam {sayac} kare alindi.")
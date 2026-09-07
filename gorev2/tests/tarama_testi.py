import time
import cv2
import numpy as np
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.double_pb2 import Double

# ---------- AYARLAR ----------
IP = "tcp:127.0.0.1:5763"
IRTIFA = 25.0
HEDEF_KUZEY = 44.0
HEDEF_DOGU = 0.0
BEKLEME_SN = 60

GZ_TOPIC = ("/world/iris_runway/model/iris_with_gimbal"
            "/model/gimbal/link/pitch_link/sensor/camera/image")
GIMBAL_TOPIC = "/gimbal/cmd_pitch"

POZISYON_MASKESI = 0b0000111111111000

# ---------- GAZEBO KAMERA ----------
gz_node = Node()
son_kare = None


def kamera_geldi(msg):
    global son_kare
    kanal = msg.step // msg.width
    dizi = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, kanal))
    if kanal == 3:
        dizi = cv2.cvtColor(dizi, cv2.COLOR_RGB2BGR)
    son_kare = dizi


def gimbal_asagi():
    yayinci = gz_node.advertise(GIMBAL_TOPIC, Double)
    time.sleep(0.5)
    mesaj = Double()
    mesaj.data = 1.57
    for _ in range(10):
        yayinci.publish(mesaj)
        time.sleep(0.2)
    print("Gimbal asagi cevrildi.")


# ---------- MAVLINK ----------
def statustext_bas(captan):
    while True:
        m = captan.recv_match(type="STATUSTEXT", blocking=False)
        if m is None:
            return
        print("  [OTOPILOT]", m.text)


def mod_degistir(captan, mod_adi):
    mod_id = captan.mode_mapping()[mod_adi]
    captan.mav.command_long_send(
        captan.target_system, captan.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mod_id, 0, 0, 0, 0, 0)
    ack = captan.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print(f"Mod {mod_adi}: {ack}")


def arm_et(captan):
    for deneme in range(5):
        captan.mav.command_long_send(
            captan.target_system, captan.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        ack = captan.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        statustext_bas(captan)
        if ack and ack.result == 0:
            print("ARM edildi.")
            return True
        print(f"Arm reddedildi (deneme {deneme + 1}), tekrar deneniyor...")
        time.sleep(2)
    return False


def kalk(captan, irtifa):
    captan.mav.command_long_send(
        captan.target_system, captan.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, irtifa)
    ack = captan.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print(f"Takeoff: {ack}")


def irtifa_oku(captan):
    m = captan.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
    if m is None:
        return None
    return m.relative_alt / 1000.0


def konum_oku(captan):
    m = captan.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=3)
    if m is None:
        return None
    return (m.x, m.y, -m.z)


def hedefe_git(captan, kuzey, dogu, irtifa):
    captan.mav.set_position_target_local_ned_send(
        0, captan.target_system, captan.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POZISYON_MASKESI,
        kuzey, dogu, -irtifa,
        0, 0, 0,
        0, 0, 0,
        0, 0)


# ---------- ANA AKIS ----------
print("Baglaniliyor:", IP)
captan = mavutil.mavlink_connection(IP)

while True:
    captan.wait_heartbeat()
    if captan.target_system != 0:
        break
print(f"Baglandi. sistem={captan.target_system} bilesen={captan.target_component}")

captan.mav.request_data_stream_send(
    captan.target_system, captan.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
print("Veri akisi istendi.")

gz_node.subscribe(Image, GZ_TOPIC, kamera_geldi)
gimbal_asagi()

mod_degistir(captan, "GUIDED")

if not arm_et(captan):
    print("HATA: arm edilemedi, cikiliyor.")
    raise SystemExit(1)

kalk(captan, IRTIFA)

print("Irtifaya cikiliyor...")
while True:
    h = irtifa_oku(captan)
    if h is not None:
        print(f"  irtifa: {h:.1f} m")
        if h >= IRTIFA * 0.95:
            break
    time.sleep(0.5)
print("Irtifa tamam.")

print(f"Hedefe gidiliyor: kuzey={HEDEF_KUZEY} dogu={HEDEF_DOGU}")
varis = time.time()
while True:
    hedefe_git(captan, HEDEF_KUZEY, HEDEF_DOGU, IRTIFA)
    k = konum_oku(captan)
    if k is not None:
        fark = ((k[0] - HEDEF_KUZEY) ** 2 + (k[1] - HEDEF_DOGU) ** 2) ** 0.5
        print(f"  konum: kuzey={k[0]:.1f} dogu={k[1]:.1f} irtifa={k[2]:.1f} | kalan={fark:.1f} m")
        if fark < 2.0:
            break
    if time.time() - varis > 90:
        print("UYARI: 90 sn'de varilamadi, yine de devam ediliyor.")
        break
    time.sleep(0.5)
print("Hedefe varildi.")

print(f"{BEKLEME_SN} saniye kamera gosteriliyor (pencerede q = erken cikis)...")
bitis = time.time() + BEKLEME_SN
while time.time() < bitis:
    hedefe_git(captan, HEDEF_KUZEY, HEDEF_DOGU, IRTIFA)
    if son_kare is not None:
        cv2.imshow("kamera", son_kare)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    time.sleep(0.1)

if son_kare is not None:
    cv2.imwrite("/tmp/hedefler.png", son_kare)
    print("Kaydedildi: /tmp/hedefler.png")
else:
    print("HATA: hic kare gelmedi.")

cv2.destroyAllWindows()
print("Script bitti. Drone havada asili kaldi (GUIDED, 3 sn sonra duracak).")
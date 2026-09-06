import math
import sys
import time
from pymavlink import mavutil

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from utils.lemniscate import lemniscate_noktalari
from utils.ned_to_global import ned_to_global

# ---------- AYARLAR ----------
IP = "tcp:127.0.0.1:5763"
BASIT_TEST = False          # True = 3 nokta ucgen, False = sonsuzluk
HIZ_MS = 2.5                # WP_SPD
NOKTA_SAYISI = 50           # 2 tur -> 25 nokta/tur -> ~6.1 m aralik
BASLANGIC_FAZI = math.pi    # pi -> sekil kuzey 10'da baslar (kalkisa yakin)
# -----------------------------

WP = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
SPLINE = mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT
FRAME_REL = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
FRAME_ABS = mavutil.mavlink.MAV_FRAME_GLOBAL

captan = mavutil.mavlink_connection(IP)
while True:
    captan.wait_heartbeat()
    if captan.target_system != 0:
        break
SYS = captan.target_system
COMP = captan.target_component
print("baglandi ->", SYS, COMP)


def madde_sayisi():
    captan.mav.mission_request_list_send(SYS, COMP)
    m = captan.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    return None if m is None else m.count


# --- 1) EV NOKTASINI OKU ---
sayi = madde_sayisi()
if sayi is None:
    print("HATA: MISSION_COUNT gelmedi")
    sys.exit(1)
print("hafizada su an", sayi, "madde var")

captan.mav.mission_request_int_send(SYS, COMP, 0)
ev = captan.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
if ev is None:
    print("HATA: ev noktasi okunamadi")
    sys.exit(1)

EV_ENLEM = ev.x / 1e7
EV_BOYLAM = ev.y / 1e7
EV_IRTIFA = ev.z
print("EV NOKTASI:", EV_ENLEM, EV_BOYLAM, EV_IRTIFA)
captan.mav.mission_ack_send(SYS, COMP, 0)

# --- 2) NOKTALARI URET ---
if BASIT_TEST:
    noktalar = [(20.0, 0.0, -15.0), (50.0, 0.0, -15.0), (35.0, 15.0, -15.0)]
    print("BASIT TEST: 3 nokta")
else:
    noktalar = lemniscate_noktalari(
        nokta_sayisi=NOKTA_SAYISI,
        baslangic_fazi=BASLANGIC_FAZI)
    print("SONSUZLUK:", len(noktalar), "nokta")

kuzeyler = [n[0] for n in noktalar]
dogular = [n[1] for n in noktalar]
print("  kuzey araligi:", round(min(kuzeyler), 1), "..", round(max(kuzeyler), 1))
print("  dogu  araligi:", round(min(dogular), 1), "..", round(max(dogular), 1))
print("  ilk nokta NED:", [round(v, 1) for v in noktalar[0]])

d = math.dist(noktalar[0][:2], noktalar[1][:2])
print("  ardisik nokta araligi: ~", round(d, 2), "m (WP_RADIUS_M = 2.0)")
if d < 3.0:
    print("  UYARI: aralik kabul yaricapina cok yakin")

# --- 3) HIZI AYARLA ---
captan.mav.param_set_send(SYS, COMP, b"WP_SPD", HIZ_MS,
                          mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
for _ in range(30):
    p = captan.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if p is None:
        break
    if p.param_id.strip('\x00') == "WP_SPD":
        print("WP_SPD ->", p.param_value)
        break

# --- 4) HAFIZAYI SIL ---
captan.mav.mission_clear_all_send(SYS, COMP)
ack = captan.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
if ack is None:
    print("HATA: silme onaylanmadi")
    sys.exit(1)
print("silindi, ACK:", ack.type)
time.sleep(0.5)

# --- 5) MADDE LISTESINI KUR ---
maddeler = [(FRAME_ABS, WP, int(EV_ENLEM * 1e7),
             int(EV_BOYLAM * 1e7), EV_IRTIFA)]

for i, (kuzey, dogu, asagi) in enumerate(noktalar):
    enlem, boylam = ned_to_global(EV_ENLEM, EV_BOYLAM, kuzey, dogu)
    komut = WP if i == 0 else SPLINE
    maddeler.append((FRAME_REL, komut, int(enlem * 1e7),
                     int(boylam * 1e7), -asagi))

print("toplam", len(maddeler), "madde yuklenecek (0 = ev noktasi)")

# --- 6) EL SIKISMA ILE YUKLE ---
captan.mav.mission_count_send(SYS, COMP, len(maddeler))

gonderilen = 0
for _ in range(len(maddeler) * 3):
    istek = captan.recv_match(
        type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
        blocking=True, timeout=10)
    if istek is None:
        print("HATA: otopilot madde istemedi, gonderilen:", gonderilen)
        sys.exit(1)

    s = istek.seq
    frame, komut, x, y, z = maddeler[s]
    captan.mav.mission_item_int_send(
        SYS, COMP, s, frame, komut, 0, 1,
        0, 0, 0, 0, x, y, z)
    gonderilen += 1

    if s == len(maddeler) - 1:
        break

ack = captan.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
if ack is None:
    print("HATA: MISSION_ACK gelmedi")
    sys.exit(1)
print("MISSION_ACK:", ack.type, "(0 = kabul)")

# --- 7) DOGRULA ---
time.sleep(1.0)
son = madde_sayisi()
print("YUKLEMEDEN SONRA hafizada:", son, "madde")
if son == len(maddeler):
    print("SONUC: YUKLEME BASARILI")
else:
    print("SONUC: SAYI TUTMUYOR, beklenen", len(maddeler))
    sys.exit(1)

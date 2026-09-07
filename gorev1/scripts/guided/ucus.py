import math
import os
import sys
import time
from pymavlink import mavutil

from pathlib import Path                                    # yol islemleri icin modern kutuphane
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "ortak"))

from utils.lemniscate import lemniscate_noktalari

# ---------- AYARLAR ----------
IP = "tcp:127.0.0.1:5763"
IRTIFA = 15.0
NOKTA_SAYISI = 240          # 2 tur -> 120/tur -> ~1.28 m aralik
BASLANGIC_FAZI = math.pi    # sekil kuzey 10'da baslar (kalkisa yakin)
HZ = 2.0                    # saniyede kac hedef -> hiz = aralik x HZ
GIRIS_BEKLE = 30            # ilk noktaya varmak icin taninan sure (sn)
# -----------------------------

POZ_MASKE = 0b0000111111111000   # sadece pozisyon (giris icin)
POZ_HIZ_MASKE = 0b0000111111000000  # pozisyon + hiz (dongu icin)
FRAME = mavutil.mavlink.MAV_FRAME_LOCAL_NED

captan = mavutil.mavlink_connection(IP)
while True:
    captan.wait_heartbeat()
    if captan.target_system != 0:
        break
SYS = captan.target_system
COMP = captan.target_component
print("baglandi ->", SYS, COMP)

D = {"alt": None, "fix": None, "mod": None, "armed": 0,
     "x": None, "y": None, "z": None}


def oku(timeout=1):
    m = captan.recv_match(blocking=True, timeout=timeout)
    if m is None:
        return None
    t = m.get_type()
    if t == 'STATUSTEXT':
        print("  [OTOPILOT]", m.text)
    elif t == 'GLOBAL_POSITION_INT':
        D["alt"] = m.relative_alt / 1000.0
    elif t == 'LOCAL_POSITION_NED':
        D["x"] = m.x
        D["y"] = m.y
        D["z"] = m.z
    elif t == 'GPS_RAW_INT':
        D["fix"] = m.fix_type
    elif t == 'HEARTBEAT':
        D["mod"] = captan.flightmode
        D["armed"] = captan.motors_armed()
    return m


def bekle(kosul, saniye, etiket):
    bitis = time.time() + saniye
    son = 0
    while time.time() < bitis:
        oku()
        if kosul():
            return True
        if time.time() - son >= 3:
            son = time.time()
            print("  ...", etiket, "| alt:", D["alt"], "fix:", D["fix"],
                  "mod:", D["mod"], "armed:", D["armed"])
    return False


def hedef_gonder(kuzey, dogu, asagi):
    captan.mav.set_position_target_local_ned_send(
        0, SYS, COMP, FRAME, POZ_MASKE,
        kuzey, dogu, asagi,
        0, 0, 0,
        0, 0, 0,
        0, 0)


def hedef_hiz_gonder(kuzey, dogu, asagi, vk, vd):
    captan.mav.set_position_target_local_ned_send(
        0, SYS, COMP, FRAME, POZ_HIZ_MASKE,
        kuzey, dogu, asagi,
        vk, vd, 0,
        0, 0, 0,
        0, 0)


def uzaklik(kuzey, dogu):
    if D["x"] is None:
        return 9999.0
    return ((kuzey - D["x"]) ** 2 + (dogu - D["y"]) ** 2) ** 0.5


# --- ROTA ---
noktalar = lemniscate_noktalari(
    nokta_sayisi=NOKTA_SAYISI, baslangic_fazi=BASLANGIC_FAZI)
aralik = math.dist(noktalar[0][:2], noktalar[1][:2])
print("ROTA:", len(noktalar), "nokta | aralik", round(aralik, 2), "m",
      "| hedef hiz", round(aralik * HZ, 2), "m/s")
print("  kuzey:", round(min(n[0] for n in noktalar), 1), "..",
      round(max(n[0] for n in noktalar), 1))
print("  dogu :", round(min(n[1] for n in noktalar), 1), "..",
      round(max(n[1] for n in noktalar), 1))

# --- 0) VERI AKISI ---
captan.mav.request_data_stream_send(
    SYS, COMP, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
print("=== 0) VERI AKISI ===")
if not bekle(lambda: D["alt"] is not None and D["x"] is not None,
             10, "akis"):
    print("HATA: konum mesajlari gelmiyor")
    sys.exit(1)
print("VERI AKISI: tamam")

# --- 1) GPS ---
print("=== 1) GPS KILIDI ===")
if not bekle(lambda: D["fix"] is not None and D["fix"] >= 3, 120, "gps"):
    print("HATA: 3D fix yok")
    sys.exit(1)
print("GPS: fix", D["fix"])

# --- 2) GUIDED ---
captan.mav.set_mode_send(
    SYS, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    captan.mode_mapping()["GUIDED"])
if not bekle(lambda: D["mod"] == "GUIDED", 10, "mod"):
    print("HATA: GUIDED olmadi ->", D["mod"])
    sys.exit(1)
print("MOD: GUIDED")

# --- 3) ARM ---
print("=== 3) ARM ===")
bitis = time.time() + 60
while time.time() < bitis and not D["armed"]:
    captan.mav.command_long_send(
        SYS, COMP, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    bekle(lambda: D["armed"], 4, "arm")

if not D["armed"]:
    print("HATA: arm olmadi")
    sys.exit(1)
print("ARM: tamam")

# --- 4) TAKEOFF ---
captan.mav.command_long_send(
    SYS, COMP, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, IRTIFA)
print("TAKEOFF verildi, hedef", IRTIFA, "m")

if not bekle(lambda: D["alt"] is not None and D["alt"] >= IRTIFA * 0.95,
             60, "tirmanis"):
    print("HATA: irtifaya ulasilmadi, son:", D["alt"])
    sys.exit(1)
print("IRTIFA: tamam ->", D["alt"])
time.sleep(2)

# --- 5) GIRIS: ilk noktaya git ---
ilk_k, ilk_d, ilk_a = noktalar[0]
print("=== 5) GIRIS -> ", round(ilk_k, 1), round(ilk_d, 1), "===")

bitis = time.time() + GIRIS_BEKLE
yazim = 0
vardi = False
while time.time() < bitis:
    hedef_gonder(ilk_k, ilk_d, ilk_a)
    oku(0.2)
    kalan = uzaklik(ilk_k, ilk_d)
    if kalan < 2.0:
        vardi = True
        break
    if time.time() - yazim >= 2.0:
        yazim = time.time()
        print("  giris | gercek", round(D["x"], 1), round(D["y"], 1),
              "| kalan", round(kalan, 1), "m")

if not vardi:
    print("HATA: ilk noktaya varilamadi, kalan", round(uzaklik(ilk_k, ilk_d), 1))
    print("  -> once tek_hedef.py calistir")
    sys.exit(1)
print("GIRIS: tamam")
time.sleep(1)

# --- 6) SONSUZLUK DONGUSU ---
print("=== 6) SONSUZLUK -", NOKTA_SAYISI, "nokta,", HZ, "Hz ===")
periyot = 1.0 / HZ
basla = time.time()
yazim = 0
sapma_max = 0.0

for i, (kuzey, dogu, asagi) in enumerate(noktalar):
    dongu_basi = time.time()
    j = (i + 1) % len(noktalar)
    vk = (noktalar[j][0] - kuzey) * HZ
    vd = (noktalar[j][1] - dogu) * HZ
    hedef_hiz_gonder(kuzey, dogu, asagi, vk, vd)

    while time.time() - dongu_basi < periyot:
        oku(0.02)

    bekleme = time.time() + 10
    while uzaklik(kuzey, dogu) > 5.0 and time.time() < bekleme:
        hedef_hiz_gonder(kuzey, dogu, asagi, vk, vd)
        oku(0.1)

    sapma = uzaklik(kuzey, dogu)
    sapma_max = max(sapma_max, sapma)

    if time.time() - yazim >= 1.0:
        yazim = time.time()
        print("nokta", i, "/", len(noktalar),
              "| hedef", round(kuzey, 1), round(dogu, 1),
              "| gercek", round(D["x"], 1), round(D["y"], 1),
              "| sapma", round(sapma, 1),
              "| irtifa", D["alt"])

sure = time.time() - basla
print("=== SONUC ===")
print("sure:", round(sure, 1), "sn | beklenen:",
      round(len(noktalar) / HZ, 1), "sn")
print("ortalama hiz:", round(aralik * len(noktalar) / sure, 2), "m/s")
print("en buyuk sapma:", round(sapma_max, 2), "m")
if sapma_max > 8.0:
    print("UYARI: drone hedefi yakalayamiyor - HZ dusur")

print("iniliyor...")
captan.mav.set_mode_send(
    SYS, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    captan.mode_mapping()["RTL"])

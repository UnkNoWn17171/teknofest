"""Gorev 2 -- tarama rotasi. 3 duraktan hedefleri bulur.

MANTIK
  Kamera 25 m'den 78x58 m goruyor, tarama alani ~40x40 m. Bicerdover deseni
  gereksiz. 3 durakta durup bakmak yeterli ve her hedef birden fazla karede
  gorunur -> olcumleri MEDYANLAYIP hatayi dusururuz.

  Her durakta: hedefe git -> DUR ve YATAYA GEL -> otur -> N kare cek.

NEDEN 25 M, 30 M DEGIL  (7 Eylul 2026 kosusunda olculdu)
  ucgen alani = (kok3/4) x kenar^2 = 0.433 m^2
  30 m'de piksel alani = 0.433 x (205.47/30)^2 = 20.3 px  -> MIN_ALAN tam 20
  25 m'de piksel alani = 0.433 x (205.47/25)^2 = 29.2 px  -> rahat
  30 m'de ucgen 15 karenin sadece 6'sinda yakalandi. MIN_ALAN'i dusurmek
  yerine, esigin dogrulandigi irtifaya donuyoruz.

NEDEN OTURMA SURESI VAR
  30 m kosusunda duraga varir varmaz alinan kareler 3-5 m sapti, son kareler
  dogruydu. Gimbal stabilize degil; govde egikken cekilen kare kayiyor.

SARTNAME NOTU
  Gercek yarista tarama, 2. direk disaridan alindiktan SONRA baslar.
  Bu script su an sadece taramayi test ediyor; giris rotasi 8. adimda gelecek.
"""

import sys
import time
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import mavutil
from utils.okuyucu import Okuyucu
from utils.tespit import bul_hedefler
from utils.konum import piksel_to_ned

# ---------- AYARLAR ----------
IP = "tcp:127.0.0.1:5763"
IRTIFA = 25.0
DURAKLAR = [(55.0, 0.0), (42.0, 0.0), (30.0, 0.0)]
KARE_SAYISI = 6
ESLEME_YARICAP = 3.0
VARIS_ESIGI = 1.0
VARIS_AZAMI_SN = 60
OTURMA_SN = 3.0
AZAMI_EGIM = 1.5

POZ_MASKE = 0b0000111111111000
FRAME = mavutil.mavlink.MAV_FRAME_LOCAL_NED


# ================== YARDIMCI ==================
def medyan(dizi):
    """Ortanca deger. Ortalamadan farki: tek sapik olcum sonucu bozamaz."""
    s = sorted(dizi)
    n = len(s)
    if n == 0:
        return 0.0
    orta = n // 2
    if n % 2 == 1:
        return s[orta]
    return (s[orta - 1] + s[orta]) / 2.0


def hedef_gonder(captan, kuzey, dogu, irtifa):
    """GUIDED konum hedefi yollar. ACK DONMEZ, dogrulama konumla yapilir."""
    captan.mav.set_position_target_local_ned_send(
        0, captan.target_system, captan.target_component,
        FRAME, POZ_MASKE,
        kuzey, dogu, -irtifa,
        0, 0, 0,
        0, 0, 0,
        0, 0)


def noktaya_git(captan, oku, kuzey, dogu, irtifa, etiket):
    """Hedefe gider, durgunlugu bekler, sonra oturmasi icin bekler."""
    print("--> %s: K %s D %s" % (etiket, kuzey, dogu))
    bitis = time.time() + VARIS_AZAMI_SN
    yazim = 0
    while time.time() < bitis:
        hedef_gonder(captan, kuzey, dogu, irtifa)
        oku.kuyrugu_bosalt()
        if oku.kuzey is None:
            time.sleep(0.2)
            continue
        kalan = ((kuzey - oku.kuzey) ** 2 + (dogu - oku.dogu) ** 2) ** 0.5
        if kalan < VARIS_ESIGI:
            print("    varildi (kalan %.1f m), oturmasi bekleniyor..." % kalan)
            oku.durgunlugu_bekle(azami_egim=AZAMI_EGIM)
            # Oturma sirasinda da komut gonderilir, yoksa GUID_TIMEOUT=3 sn
            # dolar ve otopilot hedefi birakir.
            oturma_bitis = time.time() + OTURMA_SN
            while time.time() < oturma_bitis:
                hedef_gonder(captan, kuzey, dogu, irtifa)
                oku.kuyrugu_bosalt()
                time.sleep(0.2)
            print("    hazir. hiz=%.2f egim=%.2f deg irtifa=%.2f"
                  % (oku.hiz, oku.egim_derece(), oku.irtifa))
            return True
        if time.time() - yazim >= 2.0:
            yazim = time.time()
            print("    K %.1f D %.1f | kalan %.1f m"
                  % (oku.kuzey, oku.dogu, kalan))
        time.sleep(0.2)
    print("    HATA: varilamadi")
    return False


def durakta_olc(captan, oku, durak_no, kuzey, dogu):
    """Bir durakta olcum alir. Egik kareler atlanir."""
    ham = []
    for i in range(KARE_SAYISI):
        hedef_gonder(captan, kuzey, dogu, IRTIFA)

        o = oku.olcum_al()
        if o is None:
            continue

        if o["egim"] > AZAMI_EGIM or o["hiz"] > 0.3:
            print("    kare %d: egim %.2f deg hiz %.2f -- ATLANDI"
                  % (i, o["egim"], o["hiz"]))
            continue

        sayi = 0
        for b in bul_hedefler(o["kare"]):
            if b["sekil"] == "kare":
                continue
            px, py = b["merkez"]
            k, d = piksel_to_ned(px, py, o["irtifa"],
                                 o["kuzey"], o["dogu"], o["yaw"])
            ham.append({
                "renk": b["renk"], "sekil": b["sekil"],
                "kuzey": k, "dogu": d,
                "alan": b["alan"], "guven": b["guven"],
                "durak": durak_no,
                "egim": o["egim"], "irtifa": o["irtifa"],
            })
            sayi += 1
        print("    kare %d: egim %.2f deg irtifa %.2f yaw %.1f deg -> %d hedef"
              % (i, o["egim"], o["irtifa"], math.degrees(o["yaw"]), sayi))
        time.sleep(0.15)
    return ham


def kumele(ham):
    """Ayni hedefin farkli olcumlerini birlestirir, MEDYANINI alir."""
    kumeler = []
    for o in ham:
        for k in kumeler:
            if k["renk"] != o["renk"] or k["sekil"] != o["sekil"]:
                continue
            mes = ((k["kuzey"] - o["kuzey"]) ** 2 +
                   (k["dogu"] - o["dogu"]) ** 2) ** 0.5
            if mes < ESLEME_YARICAP:
                k["olcumler"].append(o)
                k["kuzey"] = medyan([x["kuzey"] for x in k["olcumler"]])
                k["dogu"] = medyan([x["dogu"] for x in k["olcumler"]])
                break
        else:
            kumeler.append({"renk": o["renk"], "sekil": o["sekil"],
                            "kuzey": o["kuzey"], "dogu": o["dogu"],
                            "olcumler": [o]})

    for k in kumeler:
        uzakliklar = [((x["kuzey"] - k["kuzey"]) ** 2 +
                       (x["dogu"] - k["dogu"]) ** 2) ** 0.5
                      for x in k["olcumler"]]
        k["olcum_sayisi"] = len(k["olcumler"])
        k["sapma"] = max(uzakliklar) if uzakliklar else 0.0
        k["alan_med"] = medyan([x["alan"] for x in k["olcumler"]])
        del k["olcumler"]

    kumeler.sort(key=lambda k: -k["olcum_sayisi"])
    return kumeler


def en_iyileri_sec(kumeler):
    """Her (renk, sekil) icin en cok olculen kumeyi tutar.

    Sahada TEK mavi altigen ve TEK kirmizi ucgen var. Ayni sekilden iki kume
    ciktiysa biri hatali olcumden dogmustur; cok olculen kazanir.
    """
    en_iyi = {}
    for k in kumeler:
        anahtar = (k["renk"], k["sekil"])
        if (anahtar not in en_iyi
                or k["olcum_sayisi"] > en_iyi[anahtar]["olcum_sayisi"]):
            en_iyi[anahtar] = k
    return sorted(en_iyi.values(), key=lambda k: -k["olcum_sayisi"])


# ================== ANA AKIS ==================
print("Baglaniliyor:", IP)
captan = mavutil.mavlink_connection(IP)
while True:
    captan.wait_heartbeat()
    if captan.target_system != 0:
        break
print("Baglandi ->", captan.target_system, captan.target_component)

captan.mav.request_data_stream_send(
    captan.target_system, captan.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

oku = Okuyucu(captan)
oku.gimbal_asagi()

print("=== GPS bekleniyor ===")
bitis = time.time() + 120
fix = None
while time.time() < bitis:
    m = captan.recv_match(type="GPS_RAW_INT", blocking=False)
    if m is not None:
        fix = m.fix_type
        if fix >= 3:
            break
    time.sleep(0.2)
if fix is None or fix < 3:
    print("HATA: 3D fix yok")
    sys.exit(1)
print("GPS: fix", fix)

captan.mav.set_mode_send(
    captan.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    captan.mode_mapping()["GUIDED"])
time.sleep(2)
oku.kuyrugu_bosalt()
print("MOD:", captan.flightmode)

print("=== ARM ===")
bitis = time.time() + 60
while time.time() < bitis and not captan.motors_armed():
    captan.mav.command_long_send(
        captan.target_system, captan.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(2)
    oku.kuyrugu_bosalt()
if not captan.motors_armed():
    print("HATA: arm olmadi")
    sys.exit(1)
print("ARM: tamam")

captan.mav.command_long_send(
    captan.target_system, captan.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, IRTIFA)
print("TAKEOFF -> %s m" % IRTIFA)
bitis = time.time() + 90
while time.time() < bitis:
    oku.kuyrugu_bosalt()
    if oku.irtifa is not None and oku.irtifa >= IRTIFA * 0.95:
        break
    time.sleep(0.3)
print("IRTIFA: %.1f m" % oku.irtifa)
time.sleep(2)

print("=== TARAMA: %d durak, durakta %d kare, %s m ==="
      % (len(DURAKLAR), KARE_SAYISI, IRTIFA))
basla = time.time()
ham = []
for i, (k, d) in enumerate(DURAKLAR, start=1):
    if not noktaya_git(captan, oku, k, d, IRTIFA, "durak %d" % i):
        continue
    bulgular = durakta_olc(captan, oku, i, k, d)
    print("    durak %d: %d ham bulgu" % (i, len(bulgular)))
    ham += bulgular

sure = time.time() - basla

print("")
print("=== HAM OLCUMLER ===")
for o in ham:
    print("  d%d %-8s %-8s K %6.2f D %6.2f alan %6.0f egim %.2f irt %.2f"
          % (o["durak"], o["renk"], o["sekil"], o["kuzey"], o["dogu"],
             o["alan"], o["egim"], o["irtifa"]))

kumeler = kumele(ham)
print("")
print("=== TUM KUMELER ===")
for h in kumeler:
    print("  %-8s %-8s K %6.2f D %6.2f | %d olcum, sapma %.2f m, alan %.0f px"
          % (h["renk"], h["sekil"], h["kuzey"], h["dogu"],
             h["olcum_sayisi"], h["sapma"], h["alan_med"]))

hedefler = en_iyileri_sec(kumeler)
print("")
print("=== SECILEN HEDEFLER ===")
for h in hedefler:
    print("  %-8s %-8s K %6.2f D %6.2f | %d olcum, sapma %.2f m"
          % (h["renk"], h["sekil"], h["kuzey"], h["dogu"],
             h["olcum_sayisi"], h["sapma"]))

GERCEK = {("mavi", "altigen"): (42.0, -8.0),
          ("kirmizi", "ucgen"): (48.0, 7.0)}
BEKLENEN_ALAN = {("mavi", "altigen"): 702, ("kirmizi", "ucgen"): 29}
print("")
print("=== DOGRULAMA ===")
for h in hedefler:
    g = GERCEK.get((h["renk"], h["sekil"]))
    if g is None:
        print("  %s %s: BEKLENMEYEN" % (h["renk"], h["sekil"]))
        continue
    hata = ((h["kuzey"] - g[0]) ** 2 + (h["dogu"] - g[1]) ** 2) ** 0.5
    ba = BEKLENEN_ALAN[(h["renk"], h["sekil"])]
    print("  %-8s %-8s HATA %.2f m | alan %.0f px (beklenen %d)"
          % (h["renk"], h["sekil"], hata, h["alan_med"], ba))

print("")
print("Tarama suresi: %.1f sn" % sure)
print("Iniliyor (RTL)...")
captan.mav.set_mode_send(
    captan.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    captan.mode_mapping()["RTL"])

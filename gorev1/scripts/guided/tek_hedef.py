# ============================================================================
# tek_hedef.py  --  EN BASIT OTONOM UCUS
#   kalk -> tek bir noktaya git -> geri don -> in
#
# Bu, otonom ucusa giris testidir. ucus.py'den ONCE bu calismalidir.
# Sekil yok, dongu yok, tek hedef. Bir sey ters giderse sebebi tektir.
#
# kullanim:
#   python3 tek_hedef.py --hedef sitl
#   python3 tek_hedef.py --hedef herelink --irtifa 4 --kuzey 8 --izle
#
# argumanlar:
#   --hedef   sitl / usb / herelink     (varsayilan sitl)
#   --irtifa  metre                     (varsayilan 5)
#   --kuzey   metre, ileri              (varsayilan 10)
#   --dogu    metre, saga               (varsayilan 0)
#   --sure    saniye, sert ucus siniri  (varsayilan 180)
#   --izle    saglik modulu ASLA kesmesin, sadece yazsin
# ============================================================================

import sys
import time
from pathlib import Path

KOK = Path(__file__).resolve().parents[3]          # .../teknofest
sys.path.insert(0, str(KOK / "ortak"))

from pymavlink import mavutil
from baglanti import baglan, hedef_sec
import guven
import saglik


# ---------- arguman yardimcisi ----------
def arg(ad, varsayilan):
    if ad in sys.argv:
        i = sys.argv.index(ad)
        if i + 1 < len(sys.argv):
            return float(sys.argv[i + 1])
    return varsayilan


IRTIFA = arg("--irtifa", 5.0)
HEDEF_KUZEY = arg("--kuzey", 10.0)
HEDEF_DOGU = arg("--dogu", 0.0)
SURE_SINIRI = arg("--sure", 180.0)
IZLE = "--izle" in sys.argv

VARIS_YARICAP = 2.0          # bu kadar yaklasinca "vardi" say
GIDIS_SURESI = 45            # hedefe varmak icin taninan sure
BEKLEME = 5                  # hedefte kac saniye dursun

POZ_MASKE = 0b0000111111111000            # sadece pozisyon
FRAME = mavutil.mavlink.MAV_FRAME_LOCAL_NED

D = {"alt": None, "fix": None, "mod": None, "armed": 0,
     "x": None, "y": None, "z": None, "ev": False}

_son_uyari = [0.0]


# ---------------------------------------------------------------- okuma
def oku(zaman_asimi=0.2):
    m = captan.recv_match(blocking=True, timeout=zaman_asimi)
    saglik.besle(m)                        # her mesaj saglik modulune de gider
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
    elif t == 'HOME_POSITION':
        D["ev"] = True
    elif t == 'HEARTBEAT':
        D["mod"] = captan.flightmode
        D["armed"] = captan.motors_armed()
    return m


def bekle(kosul, saniye, etiket):
    bitis = time.time() + saniye
    son = 0.0
    while time.time() < bitis:
        oku()
        if kosul():
            return True
        if time.time() - son >= 3:
            son = time.time()
            print("  ...", etiket, "| alt:", D["alt"], "fix:", D["fix"],
                  "mod:", D["mod"], "armed:", D["armed"])
    return False


def uzaklik(kuzey, dogu):
    if D["x"] is None:
        return 9999.0
    return ((kuzey - D["x"]) ** 2 + (dogu - D["y"]) ** 2) ** 0.5


def hedef_gonder(kuzey, dogu, asagi):
    captan.mav.set_position_target_local_ned_send(
        0, SYS, COMP, FRAME, POZ_MASKE,
        kuzey, dogu, asagi,
        0, 0, 0,
        0, 0, 0,
        0, 0)


def saglik_bak():
    """0 = devam, 2 = gorevi bitir (RTL), 3 = acil in (LAND)"""
    d = saglik.kontrol()
    if d["seviye"] == 0:
        return 0
    if d["seviye"] == 1:
        if time.time() - _son_uyari[0] >= 5:
            _son_uyari[0] = time.time()
            print("  [UYARI]", d["sebep"])
        return 0
    print("\n" + "!" * 60)
    print("[%s] %s" % (d["kademe"], d["sebep"]))
    for x in d["hepsi"][1:]:
        print("        + %s" % x)
    print("!" * 60)
    return d["seviye"]


def sure_bak():
    if guven.sure_doldu():
        print("\n[BITIR] ucus sure siniri doldu")
        return 2
    return 0


# ================================================================== BASLA
print("=" * 62)
print("TEK HEDEF TESTI")
print("  irtifa %.1f m | hedef kuzey %.1f dogu %.1f | sure siniri %.0f sn"
      % (IRTIFA, HEDEF_KUZEY, HEDEF_DOGU, SURE_SINIRI))
print("  toplam yaricap gereken saha: ~%.0f m" % (abs(HEDEF_KUZEY) + 10))
print("=" * 62)

captan, SYS, COMP = baglan(hedef_sec("sitl"))

guven.kur(captan, SYS, COMP, cikis_modu="LAND")
guven.sure_baslat(SURE_SINIRI)
saglik.sifirla()
saglik.mod_ayarla("izle" if IZLE else "koru")

gorev_kesildi = False
acil = False

try:
    # --- 1) VERI AKISI ---
    print("\n=== 1) VERI AKISI ===")
    captan.mav.command_long_send(SYS, COMP,
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                 242, 0, 0, 0, 0, 0, 0)     # HOME_POSITION
    if not bekle(lambda: D["alt"] is not None and D["x"] is not None,
                 15, "akis"):
        raise SystemExit("HATA: konum mesajlari gelmiyor.\n"
                         "  LOCAL_POSITION_NED akisini kontrol et "
                         "(python3 ortak/baglanti.py --hedef ...)")
    print("VERI AKISI: tamam | LOCAL_NED x=%.1f y=%.1f" % (D["x"], D["y"]))

    # --- 2) GPS ---
    print("\n=== 2) GPS KILIDI ===")
    if not bekle(lambda: D["fix"] is not None and D["fix"] >= 3, 120, "gps"):
        raise SystemExit("HATA: 3D fix yok")
    print("GPS: fix", D["fix"])

    # --- 3) EV NOKTASI ---
    print("\n=== 3) EV NOKTASI ===")
    if not bekle(lambda: D["ev"], 20, "ev"):
        print("UYARI: ev noktasi gelmedi -> cikista RTL yerine LAND kullanilacak")
    else:
        print("EV NOKTASI: tamam")

    # --- 4) GUIDED ---
    print("\n=== 4) GUIDED ===")
    if not guven.mod_degistir(captan, SYS, COMP, "GUIDED", 10):
        raise SystemExit("HATA: GUIDED moduna gecilemedi (su an: %s)" % D["mod"])
    print("MOD: GUIDED")

    # --- 5) ARM ---
    print("\n=== 5) ARM ===")
    if not guven.arm_et(captan, SYS, COMP, 60):
        raise SystemExit("HATA: arm olmadi. Yukaridaki [OTOPILOT] satirlari sebebi yazar.")
    print("ARM: tamam")

    # --- 6) TAKEOFF ---
    print("\n=== 6) TAKEOFF -> %.1f m ===" % IRTIFA)
    captan.mav.command_long_send(
        SYS, COMP, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, IRTIFA)
    if not bekle(lambda: D["alt"] is not None and D["alt"] >= IRTIFA * 0.95,
                 60, "tirmanis"):
        raise SystemExit("HATA: irtifaya ulasilmadi, son: %s" % D["alt"])
    print("IRTIFA: tamam ->", round(D["alt"], 1))

    # --- 7) HEDEFE GIT ---
    print("\n=== 7) HEDEFE GIT -> kuzey %.1f dogu %.1f ===" % (HEDEF_KUZEY, HEDEF_DOGU))
    bitis = time.time() + GIDIS_SURESI
    yazim = 0.0
    vardi = False
    while time.time() < bitis:
        hedef_gonder(HEDEF_KUZEY, HEDEF_DOGU, -IRTIFA)
        oku(0.2)

        s = max(saglik_bak(), sure_bak())
        if s >= 2:
            gorev_kesildi = True
            acil = (s == 3)
            break

        kalan = uzaklik(HEDEF_KUZEY, HEDEF_DOGU)
        if kalan < VARIS_YARICAP:
            vardi = True
            break
        if time.time() - yazim >= 2.0:
            yazim = time.time()
            print("  gidis | gercek %.1f %.1f | kalan %.1f m | alt %.1f | %s"
                  % (D["x"], D["y"], kalan, D["alt"] or 0, saglik.ozet()))

    if not gorev_kesildi:
        if vardi:
            print("HEDEFE VARILDI | sapma %.2f m"
                  % uzaklik(HEDEF_KUZEY, HEDEF_DOGU))
            print("  %d sn bekleniyor..." % BEKLEME)
            son = time.time() + BEKLEME
            while time.time() < son:
                hedef_gonder(HEDEF_KUZEY, HEDEF_DOGU, -IRTIFA)
                oku(0.2)
                s = max(saglik_bak(), sure_bak())
                if s >= 2:
                    gorev_kesildi = True
                    acil = (s == 3)
                    break
        else:
            print("HEDEFE VARILAMADI | kalan %.1f m"
                  % uzaklik(HEDEF_KUZEY, HEDEF_DOGU))
            gorev_kesildi = True

finally:
    # --- 8) BITIS ---
    print("\n=== 8) BITIS ===")
    if guven.acil_calisti():
        # Guvenlik agi zaten LAND'e soktu. Uzerine RTL yazarsak agin
        # verdigi acil karari eziyoruz -- RTL once 15 m'ye TIRMANIR.
        print("guvenlik agi zaten devreye girdi -> mod DEGISTIRILMIYOR")
        bekle(lambda: not D["armed"], 120, "inis")
        print("disarm:", not D["armed"])
        print("\n" + saglik.rapor())
        print("\nsonuc: GOREV KESILDI (guvenlik agi)")
        raise SystemExit(1)

    mod = "LAND" if (acil or not D["ev"]) else "RTL"
    print("secilen cikis modu:", mod,
          "(acil)" if acil else ("(ev noktasi yok)" if not D["ev"] else ""))

    if guven.mod_degistir(captan, SYS, COMP, mod, 10):
        print("%s moduna gecildi ve DOGRULANDI" % mod)
        guven.iptal()                      # ag tekrar tetiklenmesin
        bekle(lambda: not D["armed"], 90, "inis")
        print("disarm:", not D["armed"])
    else:
        print("UYARI: %s moduna gecilemedi -- guvenlik agi devreye girecek" % mod)

    print("\n" + saglik.rapor())
    print("\nsonuc:", "GOREV KESILDI" if gorev_kesildi else "GOREV TAMAM")

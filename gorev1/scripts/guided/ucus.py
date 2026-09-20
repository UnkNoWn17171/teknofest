# ============================================================================
# ucus.py  --  GUIDED modda sonsuzluk (lemniscate) ucusu   SURUM 2
#
# SURUM 1'DEN FARKLAR (hepsi olculen bir soruna karsilik geliyor):
#   1. IP sabit degil        -> --hedef sitl / usb / herelink
#   2. LOCAL_POSITION_NED    -> baglanti.py acikca istiyor (Herelink'te GELMIYORDU)
#   3. Cikis garantisi       -> guven.py: Ctrl+C, hata, kill -> LAND + DOGRULAMA
#   4. Sure siniri           -> BATT_MONITOR=0, batarya izlenemiyor
#   5. Canli saglik izleme   -> saglik.py: motor dengesizligi, titresim, yaw kacisi
#   6. Saha kontrolu         -> sekil sahaya sigmiyorsa KALKMAZ
#   7. Mod degisimleri       -> hepsi dogrulaniyor (sondaki RTL dahil)
#   8. Ev noktasi yoksa      -> cikista RTL yerine LAND
#
# kullanim:
#   python3 ucus.py --hedef sitl
#   python3 ucus.py --hedef herelink --kucuk --saha 45x25 --izle
#
# argumanlar:
#   --hedef    sitl / usb / herelink       (varsayilan sitl)
#   --kucuk    kucuk saha onayari yukler   (5 m irtifa, 12x6 m sekil)
#   --irtifa   metre                        (varsayilan 15)
#   --merkez   sekil merkezi, kuzey metre   (varsayilan 35)
#   --uzunluk  yari uzunluk, metre          (varsayilan 25)
#   --genislik yari genislik, metre         (varsayilan 12.5)
#   --nokta    kac nokta                    (varsayilan 240)
#   --tur      kac tur                      (varsayilan 2)
#   --hz       saniyede kac hedef           (varsayilan 2.0)
#   --sure     sert ucus sinir, saniye      (varsayilan 300)
#   --saha     KUZEYxDOGU metre, orn 45x25  (verilmezse kontrol yapilmaz)
#   --izle     saglik modulu ASLA kesmesin
# ============================================================================

import sys
import math
import time
from pathlib import Path

KOK = Path(__file__).resolve().parents[3]          # .../teknofest
sys.path.insert(0, str(KOK / "ortak"))

from pymavlink import mavutil
from utils.lemniscate import lemniscate_noktalari
from baglanti import baglan, hedef_sec
import guven
import saglik


def arg(ad, varsayilan):
    if ad in sys.argv:
        i = sys.argv.index(ad)
        if i + 1 < len(sys.argv):
            return float(sys.argv[i + 1])
    return varsayilan


def arg_metin(ad, varsayilan=None):
    if ad in sys.argv:
        i = sys.argv.index(ad)
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return varsayilan


# ---------- AYARLAR ----------
KUCUK = "--kucuk" in sys.argv
IZLE = "--izle" in sys.argv
ZORLA = "--zorla" in sys.argv

if KUCUK:                                  # kucuk saha / gece / ilk test onayari
    IRTIFA = arg("--irtifa", 5.0)
    MERKEZ = arg("--merkez", 8.0)
    UZUNLUK = arg("--uzunluk", 6.0)
    GENISLIK = arg("--genislik", 3.0)
    NOKTA = int(arg("--nokta", 60))
    TUR = int(arg("--tur", 1))
    HZ = arg("--hz", 1.5)
else:
    IRTIFA = arg("--irtifa", 15.0)
    MERKEZ = arg("--merkez", 35.0)
    UZUNLUK = arg("--uzunluk", 25.0)
    GENISLIK = arg("--genislik", 12.5)
    NOKTA = int(arg("--nokta", 240))
    TUR = int(arg("--tur", 2))
    HZ = arg("--hz", 2.0)

SURE_SINIRI = arg("--sure", 300.0)
BASLANGIC_FAZI = math.pi                   # sekil en yakin uctan baslar
GIRIS_BEKLE = 40
GIRIS_YARICAP = 2.0
SAPMA_UYARI = 8.0
# -----------------------------

POZ_MASKE = 0b0000111111111000             # sadece pozisyon (giris icin)
POZ_HIZ_MASKE = 0b0000111111000000         # pozisyon + hiz (dongu icin)
FRAME = mavutil.mavlink.MAV_FRAME_LOCAL_NED

D = {"alt": None, "fix": None, "mod": None, "armed": 0,
     "x": None, "y": None, "z": None, "ev": False}
_son_uyari = [0.0]


# ================================================================== ROTA
noktalar = lemniscate_noktalari(
    merkez_kuzey=MERKEZ, yari_uzunluk=UZUNLUK, yari_genislik=GENISLIK,
    irtifa_m=IRTIFA, nokta_sayisi=NOKTA, tur_sayisi=TUR,
    baslangic_fazi=BASLANGIC_FAZI)

aralik = math.dist(noktalar[0][:2], noktalar[1][:2])
k_min = min(n[0] for n in noktalar)
k_max = max(n[0] for n in noktalar)
d_min = min(n[1] for n in noktalar)
d_max = max(n[1] for n in noktalar)

# Drone 0,0'dan kalkip sekle gidiyor -> ayak izi 0'i da kapsar
ayak_kuzey = max(k_max, 0.0) - min(k_min, 0.0)
ayak_dogu = max(d_max, 0.0) - min(d_min, 0.0)

print("=" * 62)
print("SONSUZLUK UCUSU" + ("  [KUCUK ONAYAR]" if KUCUK else ""))
print("  irtifa %.1f m | %d nokta | %d tur | %.1f Hz" % (IRTIFA, NOKTA, TUR, HZ))
print("  nokta araligi %.2f m -> hedef hiz %.2f m/s" % (aralik, aralik * HZ))
print("  sekil  kuzey %.1f .. %.1f | dogu %.1f .. %.1f" % (k_min, k_max, d_min, d_max))
print("  GEREKEN SAHA (kalkis noktasi dahil): %.0f m kuzey x %.0f m dogu"
      % (ayak_kuzey, ayak_dogu))
print("  tahmini sure: %.0f sn" % (NOKTA / HZ))
print("=" * 62)

# --- SAHA KONTROLU ---
saha = arg_metin("--saha")
if saha:
    try:
        sk, sd = [float(x) for x in saha.lower().split("x")]
    except ValueError:
        raise SystemExit("HATA: --saha formati  KUZEYxDOGU  olmali, orn: 45x25")
    print("\nSAHA KONTROLU: elimizdeki saha %.0f x %.0f m" % (sk, sd))
    pay_k = sk - ayak_kuzey
    pay_d = sd - ayak_dogu
    print("  kuzey pay: %+.1f m | dogu pay: %+.1f m" % (pay_k, pay_d))
    if pay_k < 5 or pay_d < 5:
        print("\n  *** SEKIL SAHAYA SIGMIYOR (her yonde en az 5 m pay lazim) ***")
        print("  cozum: --kucuk kullan, ya da --uzunluk / --genislik kucult")
        if not ZORLA:
            raise SystemExit("kalkis iptal. yine de denemek icin --zorla ekle.")
        print("  --zorla verildi, devam ediliyor")
    else:
        print("  saha yeterli")
else:
    print("\n(--saha verilmedi, saha kontrolu YAPILMADI)")


# ================================================================== BAGLAN
captan, SYS, COMP = baglan(hedef_sec("sitl"))

guven.kur(captan, SYS, COMP, cikis_modu="LAND")
guven.sure_baslat(SURE_SINIRI)
saglik.sifirla()
saglik.mod_ayarla("izle" if IZLE else "koru")


def oku(zaman_asimi=0.2):
    m = captan.recv_match(blocking=True, timeout=zaman_asimi)
    saglik.besle(m)
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
        kuzey, dogu, asagi, 0, 0, 0, 0, 0, 0, 0, 0)


def hedef_hiz_gonder(kuzey, dogu, asagi, vk, vd):
    captan.mav.set_position_target_local_ned_send(
        0, SYS, COMP, FRAME, POZ_HIZ_MASKE,
        kuzey, dogu, asagi, vk, vd, 0, 0, 0, 0, 0, 0)


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


gorev_kesildi = False
acil = False
sapma_max = 0.0
tamamlanan = 0

try:
    # --- 1) VERI AKISI ---
    print("\n=== 1) VERI AKISI ===")
    captan.mav.command_long_send(SYS, COMP,
                                 mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                 242, 0, 0, 0, 0, 0, 0)
    if not bekle(lambda: D["alt"] is not None and D["x"] is not None, 15, "akis"):
        raise SystemExit("HATA: konum mesajlari gelmiyor.\n"
                         "  once calistir: python3 ortak/baglanti.py --hedef ...")
    print("VERI AKISI: tamam | LOCAL_NED x=%.1f y=%.1f" % (D["x"], D["y"]))

    # yerdeyken LOCAL_NED sifira yakin olmali, yoksa sekil baska yerde cizilir
    r = math.hypot(D["x"], D["y"])
    if r > 3.0:
        print("UYARI: yerdeyken LOCAL_NED %.1f m uzakta -- EKF orijini burasi degil" % r)
        if not ZORLA:
            raise SystemExit("kalkis iptal. once onucus.py calistir, "
                             "ya da --zorla ekle.")

    # --- 2) GPS ---
    print("\n=== 2) GPS KILIDI ===")
    if not bekle(lambda: D["fix"] is not None and D["fix"] >= 3, 120, "gps"):
        raise SystemExit("HATA: 3D fix yok")
    print("GPS: fix", D["fix"])

    # --- 3) EV NOKTASI ---
    print("\n=== 3) EV NOKTASI ===")
    if not bekle(lambda: D["ev"], 20, "ev"):
        print("UYARI: ev noktasi yok -> cikista RTL yerine LAND kullanilacak")
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
        raise SystemExit("HATA: arm olmadi. [OTOPILOT] satirlari sebebi yazar.")
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
    time.sleep(2)

    # --- 7) GIRIS ---
    ilk_k, ilk_d, ilk_a = noktalar[0]
    print("\n=== 7) GIRIS -> kuzey %.1f dogu %.1f ===" % (ilk_k, ilk_d))
    bitis = time.time() + GIRIS_BEKLE
    yazim = 0.0
    vardi = False
    while time.time() < bitis:
        hedef_gonder(ilk_k, ilk_d, ilk_a)
        oku(0.2)
        s = max(saglik_bak(), sure_bak())
        if s >= 2:
            gorev_kesildi = True
            acil = (s == 3)
            break
        kalan = uzaklik(ilk_k, ilk_d)
        if kalan < GIRIS_YARICAP:
            vardi = True
            break
        if time.time() - yazim >= 2.0:
            yazim = time.time()
            print("  giris | gercek %.1f %.1f | kalan %.1f m | %s"
                  % (D["x"], D["y"], kalan, saglik.ozet()))

    if not gorev_kesildi and not vardi:
        print("HATA: ilk noktaya varilamadi, kalan %.1f m"
              % uzaklik(ilk_k, ilk_d))
        print("  -> once tek_hedef.py calistir")
        gorev_kesildi = True
    elif vardi:
        print("GIRIS: tamam | sapma %.2f m" % uzaklik(ilk_k, ilk_d))

    # --- 8) SONSUZLUK DONGUSU ---
    if not gorev_kesildi:
        print("\n=== 8) SONSUZLUK - %d nokta, %.1f Hz ===" % (NOKTA, HZ))
        periyot = 1.0 / HZ
        basla = time.time()
        yazim = 0.0

        for i, (kuzey, dogu, asagi) in enumerate(noktalar):
            dongu_basi = time.time()
            j = (i + 1) % len(noktalar)
            vk = (noktalar[j][0] - kuzey) * HZ
            vd = (noktalar[j][1] - dogu) * HZ
            hedef_hiz_gonder(kuzey, dogu, asagi, vk, vd)

            while time.time() - dongu_basi < periyot:
                oku(0.02)

            s = max(saglik_bak(), sure_bak())
            if s >= 2:
                gorev_kesildi = True
                acil = (s == 3)
                print("  -> %d / %d noktada kesildi" % (i + 1, len(noktalar)))
                break

            # drone geride kaldiysa biraz bekle (ama saglik kontrolu devam etsin)
            bekleme = time.time() + 10
            while uzaklik(kuzey, dogu) > 5.0 and time.time() < bekleme:
                hedef_hiz_gonder(kuzey, dogu, asagi, vk, vd)
                oku(0.1)
                if max(saglik_bak(), sure_bak()) >= 2:
                    break

            sapma = uzaklik(kuzey, dogu)
            sapma_max = max(sapma_max, sapma)
            tamamlanan = i + 1

            if time.time() - yazim >= 1.0:
                yazim = time.time()
                print("nokta %d/%d | hedef %.1f %.1f | gercek %.1f %.1f "
                      "| sapma %.1f | alt %.1f | %s"
                      % (i, len(noktalar), kuzey, dogu, D["x"], D["y"],
                         sapma, D["alt"] or 0, saglik.ozet()))

        sure = time.time() - basla
        print("\n--- DONGU SONUCU ---")
        print("tamamlanan nokta : %d / %d" % (tamamlanan, len(noktalar)))
        print("sure             : %.1f sn (beklenen %.1f)"
              % (sure, len(noktalar) / HZ))
        if sure > 0:
            print("ortalama hiz     : %.2f m/s" % (aralik * tamamlanan / sure))
        print("en buyuk sapma   : %.2f m" % sapma_max)
        if sapma_max > SAPMA_UYARI:
            print("UYARI: drone hedefi yakalayamiyor -> --hz dusur")

finally:
    # --- 9) BITIS ---
    print("\n=== 9) BITIS ===")
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
        guven.iptal()
        bekle(lambda: not D["armed"], 120, "inis")
        print("disarm:", not D["armed"])
    else:
        print("UYARI: %s moduna gecilemedi -- guvenlik agi devreye girecek" % mod)

    print("\n" + saglik.rapor())
    print("\nsonuc:", "GOREV KESILDI" if gorev_kesildi else "GOREV TAMAM")

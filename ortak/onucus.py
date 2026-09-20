# ============================================================================
# onucus.py  --  UCUS ONCESI KAPI
#
# Ucmadan once calistirilir. Hicbir sey degistirmez, sadece OKUR ve raporlar.
# Tek istisna: --mod-testi verilirse GUIDED'e gecip geri doner (disarm halde).
#
# kullanim:
#     python3 onucus.py --hedef herelink
#     python3 onucus.py --hedef sitl --mod-testi
#
# Cikis kodu: 0 = ucusa hazir, 1 = engel var
# ============================================================================

import sys
import time
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pymavlink import mavutil
from baglanti import baglan, hedef_sec
import guven

# ---------- ESIKLER ----------
# ENGEL (DUR) sadece scripti KESIN bozacak seyler icin.
# Geri kalani UYARI: bilgilendirir ama kalkisi engellemez.
# ArduPilot'un kendi on-kontrollerini TEKRARLAMIYORUZ -- o zaten arm etmez.
UYDU_UYARI    = 10
UYDU_DUR      = 6          # AHRS_GPS_MINSATS=6, ArduPilot zaten altinda arm etmez
HDOP_UYARI    = 1.3        # SITL 1.21 veriyor, bosuna uyari olmasin
HDOP_DUR      = 1.5        # GPS_HDOP_GOOD=1.4, ArduPilot zaten engelliyor
EKF_UYARI     = 0.5
EKF_DUR       = 0.8        # FS_EKF_THRESH ile ayni
PUSULA_UYARI  = 1.5        # log_28'de 3.19 ile BASARIYLA ucmustu, engellemiyoruz
MOTOR_FARK_MAX = 30        # disarm halinde cikislar birbirine bu kadar yakin olmali
ORIGIN_MESAFE_MAX = 5.0    # EKF orijini ile ev noktasi arasi (metre)
YEREL_SIFIR_MAX = 3.0      # yerdeyken LOCAL_NED x,y bu kadar yakin olmali
TOPLAMA = 20               # kac saniye veri toplayalim
# -----------------------------

# Yedekten bilinen beklenen degerler. Farkli cikarsa haber veriyoruz.
BEKLENEN = {
    "FLTMODE_CH": 6, "FLTMODE1": 16, "FLTMODE2": 0, "FLTMODE3": 9,
    "FLTMODE4": 3, "FLTMODE5": 9, "FLTMODE6": 9,
    "ARMING_RUDDER": 2, "ARMING_SKIPCHK": 1,
    "BATT_MONITOR": 0, "FS_THR_ENABLE": 0, "FS_GCS_ENABLE": 0,
    "FENCE_ENABLE": 0, "GUID_TIMEOUT": 3, "GUID_OPTIONS": 0,
    "MAV_GCS_SYSID": 255, "RC7_OPTION": 154, "INS_HNTCH_ENABLE": 0,
}

SONUC = []       # (durum, baslik, aciklama)  durum: "OK" / "UYARI" / "DUR"


def yaz(durum, baslik, aciklama=""):
    SONUC.append((durum, baslik, aciklama))
    isaret = {"OK": "  [OK]   ", "UYARI": "  [UYARI]", "DUR": "  [DUR]  "}[durum]
    print("%s %-30s %s" % (isaret, baslik, aciklama))


def mesafe(lat1, lon1, lat2, lon2):
    """Iki koordinat arasi metre (kucuk mesafeler icin duz yaklasim)."""
    dlat = math.radians(lat2 - lat1) * 6378137.0
    dlon = math.radians(lon2 - lon1) * 6378137.0 * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)


def parametre_oku(captan, sys_id, comp_id, isimler, saniye=20):
    """Belirli parametreleri tek tek ister. Tum listeyi indirmez (hizli)."""
    kalan = set(isimler)
    deger = {}
    son = 0.0
    bitis = time.time() + saniye
    while time.time() < bitis and kalan:
        if time.time() - son >= 2:
            son = time.time()
            for ad in list(kalan):
                captan.mav.param_request_read_send(
                    sys_id, comp_id, ad.encode('utf-8'), -1)
                time.sleep(0.02)
        m = captan.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if m is None:
            continue
        ad = m.param_id
        if isinstance(ad, bytes):
            ad = ad.decode('utf-8', 'ignore')
        ad = ad.strip('\x00')
        if ad in kalan:
            deger[ad] = m.param_value
            kalan.discard(ad)
    return deger, sorted(kalan)


def main():
    hedef = hedef_sec("sitl")
    mod_testi = "--mod-testi" in sys.argv

    print("=" * 62)
    print("UCUS ONCESI KONTROL  |  hedef:", hedef)
    print("=" * 62)

    captan, SYS, COMP = baglan(hedef)

    # HOME ve ORIGIN'i acikca iste
    for msg_id in (242, 49):          # HOME_POSITION, GPS_GLOBAL_ORIGIN
        captan.mav.command_long_send(
            SYS, COMP, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            msg_id, 0, 0, 0, 0, 0, 0)
        time.sleep(0.2)

    print("\n--- %d sn veri toplaniyor, drone'a dokunma ---" % TOPLAMA)
    son = {}
    metin = []
    hb = 0
    bitis = time.time() + TOPLAMA
    while time.time() < bitis:
        m = captan.recv_match(blocking=True, timeout=1)
        if m is None:
            continue
        t = m.get_type()
        if t == 'BAD_DATA':
            continue
        son[t] = m
        if t == 'HEARTBEAT':
            hb += 1
        elif t == 'STATUSTEXT' and m.text not in metin:
            metin.append(m.text)

    print("\n--- SONUCLAR ---")

    # 1) Kritik mesajlar geliyor mu
    for ad in ('LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT',
               'SERVO_OUTPUT_RAW', 'VIBRATION', 'EKF_STATUS_REPORT',
               'RC_CHANNELS', 'ATTITUDE'):
        if ad in son:
            yaz("OK", "mesaj: " + ad)
        else:
            yaz("DUR", "mesaj: " + ad, "gelmiyor -- script bu mesaja bagimli")

    # 2) Arm durumu
    if captan.motors_armed():
        yaz("DUR", "arm durumu", "ARAC ARMED! kontrol disarm halde yapilir")
    else:
        yaz("OK", "arm durumu", "disarm")

    # 3) GPS
    g = son.get('GPS_RAW_INT')
    if g is None:
        yaz("DUR", "GPS", "veri yok")
    else:
        hdop = g.eph / 100.0
        if g.fix_type < 3:
            yaz("DUR", "GPS kilidi", "fix=%d -- Guided/PosHold moduna girilemez" % g.fix_type)
        else:
            yaz("OK", "GPS kilidi", "fix=%d" % g.fix_type)
        if g.satellites_visible < UYDU_DUR:
            yaz("DUR", "uydu sayisi", "%d (ArduPilot zaten arm etmez)" % g.satellites_visible)
        elif g.satellites_visible < UYDU_UYARI:
            yaz("UYARI", "uydu sayisi", "%d (rahat olmak icin >=%d)" % (g.satellites_visible, UYDU_UYARI))
        else:
            yaz("OK", "uydu sayisi", str(g.satellites_visible))
        if hdop > HDOP_DUR:
            yaz("DUR", "GPS HDOP", "%.2f (GPS_HDOP_GOOD=1.4)" % hdop)
        elif hdop > HDOP_UYARI:
            yaz("UYARI", "GPS HDOP", "%.2f (ideali <%.1f)" % (hdop, HDOP_UYARI))
        else:
            yaz("OK", "GPS HDOP", "%.2f" % hdop)

    # 4) EKF
    e = son.get('EKF_STATUS_REPORT')
    if e is None:
        yaz("DUR", "EKF", "veri yok")
    else:
        for ad, deg in (("EKF hiz varyansi", e.velocity_variance),
                        ("EKF konum varyansi", e.pos_horiz_variance),
                        ("EKF yukseklik varyansi", e.pos_vert_variance)):
            if deg > EKF_DUR:
                yaz("DUR", ad, "%.3f (FS_EKF_THRESH=%.1f)" % (deg, EKF_DUR))
            elif deg > EKF_UYARI:
                yaz("UYARI", ad, "%.3f" % deg)
            else:
                yaz("OK", ad, "%.3f" % deg)
        # Pusula varyansi ENGEL degil: log_28'de 3.19 ile basariyla ucmustu
        if e.compass_variance > PUSULA_UYARI:
            yaz("UYARI", "EKF pusula varyansi",
                "%.3f (20 Eyl'de 3.19 ile ucmustu, engel degil)" % e.compass_variance)
        else:
            yaz("OK", "EKF pusula varyansi", "%.3f" % e.compass_variance)

    # 5) Ev noktasi ve orijin
    h = son.get('HOME_POSITION')
    o = son.get('GPS_GLOBAL_ORIGIN')
    if h is None:
        yaz("DUR", "ev noktasi", "belirlenmemis -- RTL calismaz, arm reddedilir")
    else:
        yaz("OK", "ev noktasi", "%.7f, %.7f" % (h.latitude / 1e7, h.longitude / 1e7))
        if o is not None:
            d = mesafe(h.latitude / 1e7, h.longitude / 1e7,
                       o.latitude / 1e7, o.longitude / 1e7)
            if d > ORIGIN_MESAFE_MAX:
                yaz("DUR", "EKF orijini - ev arasi",
                    "%.1f m -- LOCAL_NED hedefleri beklemedigin yerde cikar" % d)
            else:
                yaz("OK", "EKF orijini - ev arasi", "%.1f m" % d)
        else:
            yaz("UYARI", "EKF orijini", "GPS_GLOBAL_ORIGIN gelmedi")

    # 6) Yerde LOCAL_NED sifira yakin mi
    l = son.get('LOCAL_POSITION_NED')
    if l is not None:
        r = math.hypot(l.x, l.y)
        if r > YEREL_SIFIR_MAX:
            yaz("DUR", "LOCAL_NED baslangici",
                "x=%.1f y=%.1f (%.1f m) -- orijin baska yerde" % (l.x, l.y, r))
        else:
            yaz("OK", "LOCAL_NED baslangici", "x=%.1f y=%.1f" % (l.x, l.y))

    # 7) Motor cikislari disarm halde esit mi
    s = son.get('SERVO_OUTPUT_RAW')
    if s is not None:
        c = [s.servo1_raw, s.servo2_raw, s.servo3_raw, s.servo4_raw]
        fark = max(c) - min(c)
        if fark > MOTOR_FARK_MAX:
            yaz("UYARI", "motor cikislari (disarm)", "%s fark=%d" % (c, fark))
        else:
            yaz("OK", "motor cikislari (disarm)", "%s" % c)

    # 8) Titresim tabani
    v = son.get('VIBRATION')
    if v is not None:
        yaz("OK", "titresim tabani",
            "x=%.1f y=%.1f z=%.1f (motorlar kapali, ucusta anlamli)"
            % (v.vibration_x, v.vibration_y, v.vibration_z))
        toplam_clip = v.clipping_0 + v.clipping_1 + v.clipping_2
        if toplam_clip > 0:
            yaz("UYARI", "clipping sayaci",
                "IMU0=%d IMU1=%d IMU2=%d (onceki ucustan kalma)"
                % (v.clipping_0, v.clipping_1, v.clipping_2))
        else:
            yaz("OK", "clipping sayaci", "0")

    # 9) RC kanallari -- bilinen arizalar
    r = son.get('RC_CHANNELS')
    if r is None:
        yaz("UYARI", "RC kanallari", "veri yok -- kumanda acik mi")
    else:
        ch = [getattr(r, 'chan%d_raw' % i) for i in range(1, 9)]
        yaz("OK", "RC kanallari", "ch1-8: %s" % ch)
        if ch[5] in (0, 1000, 65535):
            yaz("UYARI", "mod kanali (ch6)",
                "%d -- BILINEN ARIZA: 4 Eylul'den beri olu, donanimsal mod cikisi YOK"
                % ch[5])
        else:
            yaz("OK", "mod kanali (ch6)", str(ch[5]))
        if ch[6] > 1800:
            yaz("DUR", "arm anahtari (ch7)",
                "%d -- ANAHTAR ARM KONUMUNDA, once asagi al" % ch[6])
        else:
            yaz("OK", "arm anahtari (ch7)", str(ch[6]))

    # 10) Heartbeat hizi
    hz = hb / float(TOPLAMA)
    if hz < 0.7:
        yaz("DUR", "telemetri", "heartbeat %.1f Hz (link zayif)" % hz)
    else:
        yaz("OK", "telemetri", "heartbeat %.1f Hz" % hz)

    # 11) Kritik parametreler
    print("\n--- PARAMETRELER ---")
    deger, eksik = parametre_oku(captan, SYS, COMP, list(BEKLENEN))
    for ad in sorted(BEKLENEN):
        if ad not in deger:
            yaz("UYARI", "param " + ad, "okunamadi")
            continue
        simdi = deger[ad]
        if abs(simdi - BEKLENEN[ad]) > 1e-6:
            yaz("UYARI", "param " + ad,
                "%g  (yedekte %g idi -- degismis)" % (simdi, BEKLENEN[ad]))
        else:
            yaz("OK", "param " + ad, "%g" % simdi)

    # 12) Mod testi (istege bagli, komut gonderir)
    if mod_testi:
        print("\n--- MOD TESTI (disarm halde, zararsiz) ---")
        onceki = captan.flightmode
        if guven.mod_degistir(captan, SYS, COMP, "GUIDED", 8):
            yaz("OK", "GUIDED moduna gecis", "basarili")
            if guven.mod_degistir(captan, SYS, COMP, onceki, 8):
                yaz("OK", "eski moda donus", onceki)
            else:
                yaz("UYARI", "eski moda donus", "%s'e donulemedi" % onceki)
        else:
            yaz("DUR", "GUIDED moduna gecis",
                "reddedildi -- script Guided'e giremez")
    else:
        print("\n(mod testi atlandi -- calistirmak icin --mod-testi ekle)")

    # 13) Otopilot mesajlari
    if metin:
        print("\n--- OTOPILOT MESAJLARI ---")
        for x in metin:
            print("   -", x)

    # ---- ozet ----
    dur = [x for x in SONUC if x[0] == "DUR"]
    uyari = [x for x in SONUC if x[0] == "UYARI"]
    zorla = "--zorla" in sys.argv

    print("\n" + "=" * 62)
    if dur:
        print("SONUC: UCUSA HAZIR DEGIL  --  %d engel, %d uyari" % (len(dur), len(uyari)))
        print("\nENGELLER:")
        for _, b, a in dur:
            print("   * %-30s %s" % (b, a))
    else:
        print("SONUC: UCUSA HAZIR  --  %d uyari" % len(uyari))
    if uyari:
        print("\nUYARILAR:")
        for _, b, a in uyari:
            print("   - %-30s %s" % (b, a))
    if dur and zorla:
        print("\n--zorla verildi: %d engel GORMEZDEN GELINIYOR" % len(dur))
    print("=" * 62)

    if dur and not zorla:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

# ============================================================================
# baglanti.py  --  MAVLink baglantisini tek yerden kurar
#
# kullanim:
#     from baglanti import baglan, hedef_sec
#     captan, SYS, COMP = baglan(hedef_sec("sitl"))
#
# komut satiri:
#     python3 ucus.py --hedef sitl
#     python3 ucus.py --hedef usb
#     python3 ucus.py --hedef herelink
# ============================================================================

import sys
import time
import glob
from pymavlink import mavutil

# ---------- AYARLAR ----------
HEDEFLER = {
    "sitl":     "tcp:127.0.0.1:5763",      # SITL'in TCP portu
    "herelink": "udpin:0.0.0.0:14550",     # laptop Herelink hotspot'unda
    "kopru":    "udpin:127.0.0.1:14551",   # mavproxy ikinci cikisi
}
USB_BAUD = 115200          # USB CDC'de hukumsuz ama pymavlink bir deger istiyor
ARAC_BEKLEME = 60          # arac heartbeat'i icin en fazla bekleme (sn)
# -----------------------------

# Istenecek mesajlar: (mesaj id, kac Hz)
# NOT: MAV_DATA_STREAM_ALL kullanMIYORUZ. ArduPilot REQUEST_DATA_STREAM'i
# aldiginda hizi KALICI parametreye yaziyor (MAV2_EXTRA1 vb). Mesaj bazinda
# SET_MESSAGE_INTERVAL istemek kalici yazma yapmiyor.
AKIS = [
    (32,  10),   # LOCAL_POSITION_NED   <- scriptin x,y,z kaynagi. SART.
    (33,   4),   # GLOBAL_POSITION_INT  <- relative_alt buradan
    (30,  10),   # ATTITUDE             <- roll/pitch/yaw
    (24,   2),   # GPS_RAW_INT          <- fix_type, uydu, HDOP
    (1,    2),   # SYS_STATUS           <- sensor sagligi
    (65,   2),   # RC_CHANNELS          <- kumanda kanallari
    (36,   5),   # SERVO_OUTPUT_RAW     <- motor cikislari (denge izleme)
    (241,  2),   # VIBRATION            <- titresim + clipping
    (193,  2),   # EKF_STATUS_REPORT    <- EKF varyanslari
    (74,   4),   # VFR_HUD              <- hiz, irtifa ozeti
]


def _usb_yolu():
    """Takili ArduPilot kartinin KALICI seri yolunu bulur.
    /dev/ttyACM0 sira numarasidir, degisir. by-id yolu degismez."""
    for kalip in ("/dev/gpilot",
                  "/dev/serial/by-id/*ArduPilot*",
                  "/dev/serial/by-id/*GPILOT*"):
        bulunan = sorted(glob.glob(kalip))
        if bulunan:
            return bulunan[0]
    return None


def _adres_coz(hedef):
    """Kisa ad -> gercek baglanti adresi."""
    if hedef == "usb":
        yol = _usb_yolu()
        if yol is None:
            raise SystemExit("HATA: USB'de ArduPilot karti yok. Kablo takili mi?")
        return yol
    if hedef in HEDEFLER:
        return HEDEFLER[hedef]
    return hedef                      # dogrudan adres yazilmis olabilir


def _arac_bekle(captan, saniye):
    """Aracin heartbeat'ini bekler. Yer istasyonu heartbeat'lerini eler.
    pymavlink, MAV_TYPE_GCS ve MAV_AUTOPILOT_INVALID olan heartbeat'lerde
    target_system'i 0'da birakir -- biz de bunu kullaniyoruz."""
    bitis = time.time() + saniye
    uyardi = False
    while time.time() < bitis:
        hb = captan.wait_heartbeat(timeout=2)
        if hb is None:
            continue
        if captan.target_system != 0:
            return True
        if not uyardi:
            uyardi = True
            print("  ... sadece yer istasyonu konusuyor, arac bekleniyor")
    return False


def akis_iste(captan, sys_id, comp_id, tablo=None):
    """Her mesaj icin ayri ayri hiz ister (kalici parametre yazmadan)."""
    if tablo is None:
        tablo = AKIS
    for msg_id, hz in tablo:
        aralik_us = int(1000000.0 / hz)
        captan.mav.command_long_send(
            sys_id, comp_id,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, aralik_us, 0, 0, 0, 0, 0)
        time.sleep(0.05)              # arka arkaya bosaltmamak icin
    print("akis istendi:", len(tablo), "mesaj")


def baglan(hedef="sitl", akis=True):
    """Baglanir, araci bulur, akisi acar. (captan, SYS, COMP) dondurur."""
    adres = _adres_coz(hedef)
    print("baglaniliyor ->", adres)

    try:
        if adres.startswith("/dev/"):
            captan = mavutil.mavlink_connection(adres, baud=USB_BAUD)
        else:
            captan = mavutil.mavlink_connection(adres)
    except Exception as hata:
        raise SystemExit(
            "HATA: baglanti kurulamadi -> %s\n"
            "  sebep: %s\n"
            "  - sitl     : SITL acik mi? (ayri terminalde drone-sim)\n"
            "  - usb      : kablo takili mi, QGC portu tutuyor mu?\n"
            "  - herelink : hotspot'a bagli misin, drone'a guc var mi?"
            % (adres, hata))

    if not _arac_bekle(captan, ARAC_BEKLEME):
        raise SystemExit("HATA: arac heartbeat'i gelmedi.\n"
                         "  - drone'a guc var mi?\n"
                         "  - kumanda acik ve eslesmis mi?\n"
                         "  - QGC ayni portu tutuyor olabilir")

    sys_id = captan.target_system
    comp_id = 1                       # ucus kartinin komponent numarasi
    print("arac bulundu -> sysid", sys_id, "comp", comp_id)

    if akis:
        akis_iste(captan, sys_id, comp_id)
    return captan, sys_id, comp_id


def hedef_sec(varsayilan="sitl"):
    """Komut satirindan --hedef okur. Yoksa varsayilani dondurur."""
    if "--hedef" in sys.argv:
        i = sys.argv.index("--hedef")
        if i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return varsayilan


# ---------- kendi basina calistirilirsa: baglanti testi ----------
if __name__ == "__main__":
    captan, SYS, COMP = baglan(hedef_sec("sitl"))

    print("\n10 sn dinleniyor, hangi mesajlar geliyor:")
    sayac = {}
    bitis = time.time() + 10
    while time.time() < bitis:
        m = captan.recv_match(blocking=True, timeout=1)
        if m is None:
            continue
        t = m.get_type()
        if t == 'BAD_DATA':
            continue
        sayac[t] = sayac.get(t, 0) + 1

    for t in sorted(sayac, key=lambda k: -sayac[k]):
        print("   %-24s %4d  (~%.1f Hz)" % (t, sayac[t], sayac[t] / 10.0))

    print("\nKRITIK MESAJ KONTROLU:")
    for gerekli in ('LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT',
                    'GPS_RAW_INT', 'SERVO_OUTPUT_RAW', 'VIBRATION',
                    'EKF_STATUS_REPORT', 'RC_CHANNELS'):
        print("   %-22s %s" % (gerekli, "VAR" if gerekli in sayac else "!!! YOK"))

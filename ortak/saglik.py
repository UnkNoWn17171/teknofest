# ============================================================================
# saglik.py  --  ucus sirasinda canli saglik izleme  (SURUM 2)
#
# SURUM 1'DEKI HATAM:
#   Esikleri "kotu deger" diye belirlemistim. Ama 20 Eylul sabahki BASARILI
#   ucus (log_28, 176 sn, 10.5 m) su degerlerle ucmustu:
#       motor farki 232 us   |   EKF pusula varyansi 3.19
#   Surum 1 bu ucusu IPTAL EDERDI. Yani yanlis alarm uretiyordu.
#   Surum 2'nin esikleri, BASARIYLA BITEN ucuslarin en kotu degerlerinin
#   USTUNDE. Her esigin yaninda nereden geldigi yaziyor.
#
# ARDUPILOT ZATEN NE YAPIYOR (bunlari TEKRARLAMIYORUZ):
#   FS_EKF_ACTION=1, FS_EKF_THRESH=0.8  -> EKF bozulursa otopilot LAND yapar
#   FS_VIBE_ENABLE=1                    -> asiri titresimde otopilot devreye girer
#   FS_CRASH_CHECK=1                    -> carpma algilarsa motorlari keser
#   Bu modul otopilotun BAKMADIGI seylere bakar:
#   motor dengesizligi egilimi, GCS linki, yaw kacisi, ucus suresi.
#
# UC KADEME:
#   1 UYARI : sadece ekrana yazar, ucus devam eder
#   2 BITIR : gorevi kes, kontrollu don (RTL)
#   3 ACIL  : hemen in (LAND)
#
# IKI CALISMA MODU:
#   "koru" (varsayilan) : kademeler aktif
#   "izle"              : asla 2/3 dondurmez, sadece yazar.
#                         Ilk test ucuslarinda veri toplamak icin.
#
# KULLANIM (ana dongunun okuyucusuna baglanir, ayri okuma YAPMAZ):
#     import saglik
#     saglik.sifirla()
#     saglik.mod_ayarla("izle")
#     ...
#     m = captan.recv_match(blocking=True, timeout=1)
#     saglik.besle(m)
#     d = saglik.kontrol()
#     if d["seviye"] >= 2: ...
# ============================================================================

import time

# ---------- ESIKLER ----------
# Her satirin yaninda bu sayinin NEREDEN geldigi yaziyor.

# Motor cikis farki (m1..m4 arasi, us)
# olculen: log_14 = 51 (saglikli) | log_28 = 232 (basarili) | log_10 = 331 (basarili)
MOTOR_UYARI = 150
MOTOR_BITIR = 420          # en kotu BASARILI ucusun (331) %25 ustu
MOTOR_ACIL = 600
MOTOR_UCUS_ESIK = 1150     # bu degerin ustu "motor caliisyor"

# Titresim (en buyuk eksen)
# olculen: log_14 max 64.5 ve ucus basariyla bitti
# ArduPilot'un FS_VIBE_ENABLE=1'i zaten devrede; biz cok otesine bakiyoruz
VIBE_UYARI = 40.0
VIBE_BITIR = 90.0
VIBE_ACIL = 120.0

# Clipping artisi (ucus basina gore)
# olculen: log_14'te 216 clipping vardi, ucus basarili
CLIP_UYARI = 10
CLIP_BITIR = 600

# EKF pusula varyansi
# olculen: log_28'de 3.19, ucus basarili. ArduPilot pusula varyansi tek basina
# failsafe tetiklemiyor, o yuzden genis birakiyoruz.
PUSULA_UYARI = 1.5
PUSULA_BITIR = 5.0

# EKF konum / hiz varyansi
# ArduPilot FS_EKF_THRESH=0.8'de zaten LAND yapiyor. Biz sadece uyariyoruz.
EKF_UYARI = 0.6

# GPS
HDOP_UYARI = 1.6           # GPS_HDOP_GOOD=1.4; log_26'da 2.15 gorulmustu
HDOP_BITIR = 3.0
UYDU_UYARI = 8
UYDU_BITIR = 5             # AHRS_GPS_MINSATS=6

# Yaw kacisi (derece/sn)
# Scriptin maskesi yaw komutu GONDERMIYOR (bit 10-11 ignore). Drone
# kendiliginden doneyorsa yaw yetkisi tukeniyor demektir.
# SITL olcumu: sekle giris aninda 55.8 derece/sn gecici tepe gorildu (normal
# donus manevrasi). 30'da biraksaydik her ucusta bosuna uyari verirdi.
YAW_UYARI = 50.0
YAW_BITIR = 90.0
YAW_ACIL = 150.0

# Telemetri linki (saniye)
# NOT: link koptuysa BITIR komutu zaten gidemez. Bu kademeler, link GERI
# GELDIGINDE hemen donmesi icin. Gercek cozum FS_GCS_ENABLE parametresi.
LINK_UYARI = 3.0
LINK_BITIR = 12.0
LINK_ACIL = 25.0

# Kac kez ust uste asilirsa gercek sayalim
ART_ARDA_UYARI = 3
ART_ARDA_CIDDI = 5         # BITIR ve ACIL icin
ISINMA = 6.0               # ilk N saniye kontrol yok
# -----------------------------

KADEME_AD = {0: "IYI", 1: "UYARI", 2: "BITIR", 3: "ACIL"}

D = {}
_sayac = {}
_baslangic = [0.0]
_clip_taban = [None, None, None]
_mod = ["koru"]
_gorulen = {}              # ucus boyunca gorulen en kotu degerler


def mod_ayarla(ad):
    """'koru' = kademeler aktif. 'izle' = sadece yazar, asla kesmez."""
    if ad not in ("koru", "izle"):
        raise ValueError("mod 'koru' veya 'izle' olmali")
    _mod[0] = ad
    if ad == "izle":
        print("saglik modu -> IZLE (ucus ASLA kesilmez, sadece yazar)")
    else:
        print("saglik modu -> KORU (kademeler aktif)")


def sifirla():
    D.clear()
    D.update({
        "motor": None, "motor_fark": None, "ucuyor": False,
        "vibe": None, "clip": [0, 0, 0], "clip_artis": 0,
        "pusula": None, "konum": None, "hiz": None,
        "fix": None, "uydu": None, "hdop": None,
        "yaw_hiz": None,
        "son_hb": time.time(), "link": 0.0,
    })
    _sayac.clear()
    _gorulen.clear()
    _baslangic[0] = time.time()
    _clip_taban[0] = _clip_taban[1] = _clip_taban[2] = None


def _en_kotu(ad, deger):
    if deger is None:
        return
    if ad not in _gorulen or deger > _gorulen[ad]:
        _gorulen[ad] = deger


def besle(m):
    if m is None:
        return
    t = m.get_type()

    if t == 'HEARTBEAT':
        D["son_hb"] = time.time()

    elif t == 'SERVO_OUTPUT_RAW':
        c = [m.servo1_raw, m.servo2_raw, m.servo3_raw, m.servo4_raw]
        D["motor"] = c
        if min(c) > MOTOR_UCUS_ESIK:
            D["ucuyor"] = True
            D["motor_fark"] = max(c) - min(c)
            _en_kotu("motor_fark", D["motor_fark"])
        else:
            D["ucuyor"] = False
            D["motor_fark"] = None          # yerdeyken karsilastirma anlamsiz

    elif t == 'VIBRATION':
        D["vibe"] = (m.vibration_x, m.vibration_y, m.vibration_z)
        _en_kotu("vibe", max(D["vibe"]))
        yeni = [m.clipping_0, m.clipping_1, m.clipping_2]
        D["clip"] = yeni
        for i in range(3):
            if _clip_taban[i] is None:
                _clip_taban[i] = yeni[i]
        D["clip_artis"] = max(yeni[i] - (_clip_taban[i] or 0) for i in range(3))
        _en_kotu("clip", D["clip_artis"])

    elif t == 'EKF_STATUS_REPORT':
        D["pusula"] = m.compass_variance
        D["konum"] = m.pos_horiz_variance
        D["hiz"] = m.velocity_variance
        _en_kotu("pusula", D["pusula"])
        _en_kotu("konum", D["konum"])

    elif t == 'GPS_RAW_INT':
        D["fix"] = m.fix_type
        D["uydu"] = m.satellites_visible
        D["hdop"] = m.eph / 100.0
        _en_kotu("hdop", D["hdop"])

    elif t == 'ATTITUDE':
        D["yaw_hiz"] = abs(m.yawspeed) * 57.2958    # rad/s -> derece/sn
        _en_kotu("yaw_hiz", D["yaw_hiz"])


def _kaydet(ad, seviye):
    """Ayni bulgu ust uste yeterince gelmeden gercek saymiyoruz."""
    if seviye == 0:
        _sayac[ad] = 0
        return 0
    _sayac[ad] = _sayac.get(ad, 0) + 1
    gereken = ART_ARDA_UYARI if seviye == 1 else ART_ARDA_CIDDI
    return seviye if _sayac[ad] >= gereken else 0


def _degerlendir(ad, deger, uyari, bitir, acil, metin):
    """Tek bir olcumu uc esige gore degerlendirir. Yuksek = kotu."""
    if deger is None:
        return None
    if acil is not None and deger >= acil:
        s = 3
    elif bitir is not None and deger >= bitir:
        s = 2
    elif uyari is not None and deger >= uyari:
        s = 1
    else:
        s = 0
    sonuc = _kaydet(ad, s)
    if sonuc > 0:
        return (sonuc, metin % deger)
    return None


def kontrol():
    """{"seviye": 0-3, "kademe": "IYI"/..., "sebep": str, "hepsi": [...]}"""
    D["link"] = time.time() - D.get("son_hb", time.time())
    bulgular = []

    def ek(x):
        if x:
            bulgular.append(x)

    # --- link: isinmadan bagimsiz ---
    ek(_degerlendir("link", D["link"], LINK_UYARI, LINK_BITIR, LINK_ACIL,
                    "telemetri kayip %.1f sn"))

    if time.time() - _baslangic[0] < ISINMA:
        return _cikti([])

    ek(_degerlendir("motor", D["motor_fark"], MOTOR_UYARI, MOTOR_BITIR,
                    MOTOR_ACIL, "motor dengesizligi %.0f us (yaw arizasi)"))

    ek(_degerlendir("vibe", max(D["vibe"]) if D["vibe"] else None,
                    VIBE_UYARI, VIBE_BITIR, VIBE_ACIL, "titresim %.1f"))

    ek(_degerlendir("clip", D["clip_artis"], CLIP_UYARI, CLIP_BITIR, None,
                    "ivmeolcer clipping +%.0f"))

    ek(_degerlendir("pusula", D["pusula"], PUSULA_UYARI, PUSULA_BITIR, None,
                    "EKF pusula varyansi %.2f"))
    ek(_degerlendir("konum", D["konum"], EKF_UYARI, None, None,
                    "EKF konum varyansi %.2f (ArduPilot 0.8'de kendi iner)"))
    ek(_degerlendir("hiz", D["hiz"], EKF_UYARI, None, None,
                    "EKF hiz varyansi %.2f"))

    if D["fix"] is not None and D["fix"] < 3:
        s = _kaydet("fix", 1)
        if s:
            bulgular.append((1, "GPS 3D kilidi yok (fix=%d)" % D["fix"]))
    else:
        _kaydet("fix", 0)

    ek(_degerlendir("hdop", D["hdop"], HDOP_UYARI, HDOP_BITIR, None,
                    "GPS HDOP %.2f"))

    # uydu ters yonde: dusuk = kotu
    u = D["uydu"]
    if u is not None:
        s = 2 if u <= UYDU_BITIR else (1 if u < UYDU_UYARI else 0)
        s = _kaydet("uydu", s)
        if s:
            bulgular.append((s, "uydu sayisi %d" % u))

    ek(_degerlendir("yaw", D["yaw_hiz"], YAW_UYARI, YAW_BITIR, YAW_ACIL,
                    "yaw kacisi %.0f derece/sn (komut yokken donuyor)"))

    return _cikti(bulgular)


def _cikti(bulgular):
    if not bulgular:
        return {"seviye": 0, "kademe": "IYI", "sebep": "", "hepsi": []}
    bulgular.sort(key=lambda x: -x[0])
    seviye = bulgular[0][0]
    if _mod[0] == "izle" and seviye > 1:
        seviye = 1                          # izle modunda asla kesme
    return {"seviye": seviye,
            "kademe": KADEME_AD[seviye],
            "sebep": bulgular[0][1],
            "hepsi": [b[1] for b in bulgular]}


def ozet():
    """Tek satirlik anlik durum -- ilerleme basarken kullan."""
    v = D.get("vibe")
    return ("motor=%s vibe=%s clip=+%s pusula=%s uydu=%s hdop=%s yaw=%s link=%.1f"
            % (D.get("motor_fark"),
               ("%.0f" % max(v)) if v else "-",
               D.get("clip_artis"),
               ("%.2f" % D["pusula"]) if D.get("pusula") is not None else "-",
               D.get("uydu"),
               ("%.2f" % D["hdop"]) if D.get("hdop") is not None else "-",
               ("%.0f" % D["yaw_hiz"]) if D.get("yaw_hiz") is not None else "-",
               D.get("link", 0.0)))


def rapor():
    """Ucus bitince: bu ucusta gorulen EN KOTU degerler.
    Bir sonraki ucusun esiklerini bu tabloya bakarak ayarla."""
    if not _gorulen:
        return "saglik raporu: veri yok"
    esik = {"motor_fark": (MOTOR_UYARI, MOTOR_BITIR),
            "vibe": (VIBE_UYARI, VIBE_BITIR),
            "clip": (CLIP_UYARI, CLIP_BITIR),
            "pusula": (PUSULA_UYARI, PUSULA_BITIR),
            "konum": (EKF_UYARI, None),
            "hdop": (HDOP_UYARI, HDOP_BITIR),
            "yaw_hiz": (YAW_UYARI, YAW_BITIR)}
    satir = ["--- BU UCUSTA GORULEN EN KOTU DEGERLER ---"]
    for ad in sorted(_gorulen):
        u, b = esik.get(ad, (None, None))
        satir.append("   %-12s %8.2f    (uyari %s / bitir %s)"
                     % (ad, _gorulen[ad], u, b))
    return "\n".join(satir)

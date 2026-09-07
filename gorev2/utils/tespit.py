"""Gorev 2 hedef tespiti.

Sartname: mavi duzgun ALTIGEN (kenar 2 m) ve kirmizi eskenar UCGEN (kenar 1 m)
bizim hedeflerimiz. Sabit kanat kategorisinin mavi 4x4 ve kirmizi 2x2 KARE
hedefleri celdiricidir, ayirt edilip atlanmalidir.

Ayrim olcutu: cember orani = sekil alani / en kucuk cevreleyen daire alani.
Donmeden ve olcekten bagimsizdir.
    ucgen   0.413
    kare    0.637
    altigen 0.827

ONEMLI: Bu hatta bulaniklastirma (GaussianBlur) veya morfoloji (MORPH_OPEN,
MORPH_CLOSE) EKLENMEZ. 25 m'den ucgen sadece ~27 piksel; her iki islem de
kenarlari sisirip cember oranini 0.40'tan 0.64'e cikarir ve ucgen kare olarak
siniflanir. Gurultu filtresi MIN_ALAN ile yapilir.
"""

import cv2
import numpy as np

# ---------- HSV esikleri: 25 m irtifadan olculdu, dogrulandi ----------
# Kirmizi HSV cemberinde 0 ve 180 civarinda iki parcaya bolunur, iki aralik gerekir.
KIRMIZI_ALT1 = np.array([0, 80, 60])
KIRMIZI_UST1 = np.array([10, 255, 255])
KIRMIZI_ALT2 = np.array([170, 80, 60])
KIRMIZI_UST2 = np.array([180, 255, 255])

MAVI_ALT = np.array([100, 80, 60])
MAVI_UST = np.array([130, 255, 255])

# ---------- Karar esikleri ----------
MIN_ALAN = 20           # bundan kucuk lekeler gurultu sayilir
UCGEN_ESIK = 0.525      # 0.413 ile 0.637 ortasi
ALTIGEN_ESIK = 0.732    # 0.637 ile 0.827 ortasi


def _maskeler(kare):
    """BGR goruntuden kirmizi ve mavi ikili maskeleri uretir."""
    hsv = cv2.cvtColor(kare, cv2.COLOR_BGR2HSV)

    kirmizi = cv2.bitwise_or(
        cv2.inRange(hsv, KIRMIZI_ALT1, KIRMIZI_UST1),
        cv2.inRange(hsv, KIRMIZI_ALT2, KIRMIZI_UST2))
    mavi = cv2.inRange(hsv, MAVI_ALT, MAVI_UST)

    return kirmizi, mavi


def _sekil_karari(kontur):
    """Konturun sekil adini, cember oranini ve guven degerini dondurur."""
    alan = cv2.contourArea(kontur)
    (_, _), yaricap = cv2.minEnclosingCircle(kontur)
    if yaricap <= 0:
        return "bilinmiyor", 0.0, 0.0

    oran = alan / (np.pi * yaricap * yaricap)

    if oran < UCGEN_ESIK:
        sekil, beklenen_kose = "ucgen", 3
    elif oran < ALTIGEN_ESIK:
        sekil, beklenen_kose = "kare", 4
    else:
        sekil, beklenen_kose = "altigen", 6

    # Kose sayisi ikincil onay. Tutuyorsa guven tam, tutmuyorsa dusuk.
    cevre = cv2.arcLength(kontur, True)
    kose = len(cv2.approxPolyDP(kontur, 0.03 * cevre, True))
    guven = 1.0 if kose == beklenen_kose else 0.6

    return sekil, oran, guven


def bul_hedefler(kare):
    """Goruntudeki tum renkli sekilleri bulur.

    Doner: sozluk listesi. Her sozlukte
    renk, sekil, oran, guven, alan, merkez (piksel), hedef_mi.
    """
    kirmizi, mavi = _maskeler(kare)
    sonuc = []

    for renk_adi, maske in (("kirmizi", kirmizi), ("mavi", mavi)):
        konturlar, _ = cv2.findContours(
            maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for k in konturlar:
            alan = cv2.contourArea(k)
            if alan < MIN_ALAN:
                continue

            M = cv2.moments(k)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])       # agirlik merkezi x
            cy = int(M["m01"] / M["m00"])       # agirlik merkezi y

            sekil, oran, guven = _sekil_karari(k)

            hedef_mi = ((renk_adi == "mavi" and sekil == "altigen") or
                        (renk_adi == "kirmizi" and sekil == "ucgen"))

            sonuc.append({
                "renk": renk_adi,
                "sekil": sekil,
                "oran": round(oran, 3),
                "guven": guven,
                "alan": alan,
                "merkez": (cx, cy),
                "hedef_mi": hedef_mi,
            })

    return sonuc


def ciz(kare, bulgular):
    """Bulgulari goruntu uzerine isaretler. Sadece gorsel kontrol icin."""
    vis = kare.copy()
    for b in bulgular:
        cx, cy = b["merkez"]
        renk = (0, 255, 0) if b["hedef_mi"] else (0, 0, 255)
        etiket = f"{b['renk'][:1].upper()}-{b['sekil']} {b['oran']:.2f}"
        cv2.circle(vis, (cx, cy), 4, renk, -1)
        cv2.putText(vis, etiket, (cx - 50, cy - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, renk, 1)
    return vis
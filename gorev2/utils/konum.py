"""Kamera pikselinden yer koordinatina donusum.

KABULLER
  - Kamera gimbal ile tam asagi bakiyor (cmd_pitch = +1.57).
  - Gimbal yaw eklemine komut GONDERILMIYOR -> kamera yaw'i = drone yaw'i.
    (Gimbal yaw'a komut verirsen bu dosya yanlis hesaplar.)
  - Kare alinirken drone YATAY. Gimbal stabilize degil; drone egilirse
    goruntu kayar. Bu yuzden kare her zaman DURAKTA, hiz kesilmisken alinir.
  - Zemin duz, irtifa = yere olan dik uzaklik.

IC PARAMETRELER: TAHMIN DEGIL, OLCULDU (7 Eylul 2026)
  gz topic -e -t .../sensor/camera/camera_info
      intrinsics: fx=205.46962738  fy=205.46965599  cx=320  cy=240
      distortion: 0 0 0 0 0   -> lens bozulmasi yok, duzeltme gerekmiyor
  FOV'dan hesap 320/tan(1.0) = 205.4696 ile birebir tutuyor.

ZOOM UYARISI: modelde CameraZoomPlugin var (max_zoom 125). Zoom degisirse
fx degisir ve buradaki tum hesap bozulur. /model/gimbal/sensor/camera/zoom/
cmd_zoom topic'ine ASLA komut gonderme.
"""

import math

# ---------- Olculmus kamera ic parametreleri ----------
FX = 205.46962738          # yatay odak uzakligi, piksel
FY = 205.46965599          # dikey odak uzakligi, piksel
CX = 320.0                 # goruntu merkezi x
CY = 240.0                 # goruntu merkezi y

GORUNTU_GENISLIK = 640
GORUNTU_YUKSEKLIK = 480

# Eski kod (konum_testi.py) ODAK adini kullaniyor, kirilmasin diye duruyor.
ODAK = FX


def piksel_to_ned(px, py, irtifa, drone_kuzey, drone_dogu, yaw_rad=0.0):
    """Bir piksel noktasinin NED konumunu dondurur.

    px, py       : hedefin goruntudeki merkezi (piksel)
    irtifa       : dronenin yerden yuksekligi (m)
    drone_kuzey  : dronenin o ANDAKI NED kuzey konumu (m)
    drone_dogu   : dronenin o ANDAKI NED dogu konumu (m)
    yaw_rad      : dronenin o ANDAKI yaw'i (radyan, ATTITUDE mesajindan).
                   0 = burun kuzeye. Verilmezse 0 kabul edilir.

    Doner: (hedef_kuzey, hedef_dogu) metre, NED
    """
    # 1) Goruntu merkezinden piksel sapmasi
    sapma_x = px - CX          # saga pozitif
    sapma_y = py - CY          # ekranda ASAGI pozitif

    # 2) Benzer ucgen: yerdeki metre = irtifa * (piksel sapmasi / odak)
    metre_x = irtifa * sapma_x / FX
    metre_y = irtifa * sapma_y / FY

    # 3) Once GOVDE eksenlerine cevir (drone'un kendi ileri/sag yonu).
    #    Kamera asagi bakarken goruntunun yukarisi = burnun baktigi yon.
    ileri = -metre_y           # ekranda yukari cikmak = ileri gitmek
    sag = metre_x              # ekranda saga gitmek = saga gitmek

    # 4) Govde -> NED donusu. Yaw kadar dondur.
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    kuzey_sapma = ileri * c - sag * s
    dogu_sapma = ileri * s + sag * c

    return drone_kuzey + kuzey_sapma, drone_dogu + dogu_sapma


def metre_basina_piksel(irtifa):
    """Verilen irtifada 1 metre kac piksel eder. Boyut kontrolu icin."""
    return FX / irtifa


def kapsama(irtifa):
    """Verilen irtifada bir karenin yerde kapladigi alan.

    Doner: (yatay_metre, dikey_metre)
    Tarama duraklarini planlarken bu kullanilir.
    """
    yatay = irtifa * GORUNTU_GENISLIK / FX
    dikey = irtifa * GORUNTU_YUKSEKLIK / FY
    return yatay, dikey
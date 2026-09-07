import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
from utils.tespit import bul_hedefler
from utils.konum import piksel_to_ned, metre_basina_piksel, ODAK

# tarama_testi.py cikisindaki son konum
DRONE_KUZEY = 44.0
DRONE_DOGU = 0.0
IRTIFA = 25.0

# uret_gorev2.py'deki gercek konumlar (dogrulama icin)
GERCEK = {
    ("kirmizi", "ucgen"):   (48.0, 7.0),
    ("mavi", "altigen"):    (42.0, -8.0),
    ("kirmizi", "kare"):    (55.0, -6.0),
    ("mavi", "kare"):       (33.0, 6.0),
}

print(f"Odak uzakligi: {ODAK:.1f} piksel")
print(f"{IRTIFA} m'de 1 metre = {metre_basina_piksel(IRTIFA):.1f} piksel\n")

kare = cv2.imread("/tmp/hedefler.png")
if kare is None:
    raise SystemExit("HATA: /tmp/hedefler.png yok.")

for b in bul_hedefler(kare):
    px, py = b["merkez"]
    k, d = piksel_to_ned(px, py, IRTIFA, DRONE_KUZEY, DRONE_DOGU)

    anahtar = (b["renk"], b["sekil"])
    gk, gd = GERCEK.get(anahtar, (None, None))

    satir = (f"{b['renk']:8s} {b['sekil']:8s} piksel={b['merkez']}  "
             f"hesap=(K {k:6.1f}, D {d:6.1f})")
    if gk is not None:
        hata = ((k - gk) ** 2 + (d - gd) ** 2) ** 0.5
        satir += f"  gercek=(K {gk:6.1f}, D {gd:6.1f})  HATA={hata:5.2f} m"
    print(satir)
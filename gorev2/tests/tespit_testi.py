import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
from utils.tespit import bul_hedefler, ciz

kare = cv2.imread("/tmp/hedefler.png")
if kare is None:
    raise SystemExit("HATA: /tmp/hedefler.png yok.")

bulgular = bul_hedefler(kare)

print(f"{len(bulgular)} sekil bulundu:\n")
for b in bulgular:
    isaret = "*** HEDEF" if b["hedef_mi"] else "    celdirici"
    print(f"  {b['renk']:8s} {b['sekil']:8s} oran={b['oran']:.3f} "
          f"guven={b['guven']:.1f} alan={b['alan']:6.0f} "
          f"merkez={b['merkez']}  {isaret}")

cv2.imshow("tespit", ciz(kare, bulgular))
cv2.waitKey(0)
cv2.destroyAllWindows()
import cv2
import numpy as np

kare = cv2.imread("/tmp/hedefler.png")
if kare is None:
    raise SystemExit("HATA: /tmp/hedefler.png bulunamadi.")

hsv = cv2.cvtColor(kare, cv2.COLOR_BGR2HSV)
print(f"Goruntu boyutu: {kare.shape[1]}x{kare.shape[0]}")


def olc(isim, alt, ust):
    """Verilen HSV araliginda kac piksel var, kac ayri leke olusturuyor."""
    maske = cv2.inRange(hsv, np.array(alt), np.array(ust))
    kernel = np.ones((3, 3), np.uint8)
    maske = cv2.morphologyEx(maske, cv2.MORPH_OPEN, kernel)
    maske = cv2.morphologyEx(maske, cv2.MORPH_CLOSE, kernel)

    konturlar, _ = cv2.findContours(maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"\n--- {isim} --- toplam {cv2.countNonZero(maske)} piksel, "
          f"{len(konturlar)} leke")

    for k in sorted(konturlar, key=cv2.contourArea, reverse=True)[:6]:
        alan = cv2.contourArea(k)
        if alan < 15:
            continue
        x, y, w, h = cv2.boundingRect(k)
        cevre = cv2.arcLength(k, True)
        kose = len(cv2.approxPolyDP(k, 0.03 * cevre, True))
        doluluk = alan / (w * h) if w * h > 0 else 0
        print(f"   alan={alan:6.0f}  kutu={w:3d}x{h:3d}  merkez=({x + w // 2:3d},{y + h // 2:3d})  "
              f"kose={kose}  doluluk={doluluk:.2f}")

    return maske


# Kirmizi HSV'de 0 ve 180 civarinda iki parcaya bolunur, iki maske gerekir
k1 = cv2.inRange(hsv, np.array([0, 80, 60]), np.array([10, 255, 255]))
k2 = cv2.inRange(hsv, np.array([170, 80, 60]), np.array([180, 255, 255]))
kirmizi = cv2.bitwise_or(k1, k2)
konturlar, _ = cv2.findContours(kirmizi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
print(f"\n--- KIRMIZI --- toplam {cv2.countNonZero(kirmizi)} piksel, {len(konturlar)} leke")
for k in sorted(konturlar, key=cv2.contourArea, reverse=True)[:6]:
    alan = cv2.contourArea(k)
    if alan < 15:
        continue
    x, y, w, h = cv2.boundingRect(k)
    cevre = cv2.arcLength(k, True)
    kose = len(cv2.approxPolyDP(k, 0.03 * cevre, True))
    doluluk = alan / (w * h) if w * h > 0 else 0
    print(f"   alan={alan:6.0f}  kutu={w:3d}x{h:3d}  merkez=({x + w // 2:3d},{y + h // 2:3d})  "
          f"kose={kose}  doluluk={doluluk:.2f}")

mavi = olc("MAVI", [100, 80, 60], [130, 255, 255])

cv2.imshow("kirmizi maske", kirmizi)
cv2.imshow("mavi maske", mavi)
print("\nPencerelerde herhangi bir tusa bas...")
cv2.waitKey(0)
cv2.destroyAllWindows()
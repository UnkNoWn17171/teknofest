# ============================================================================
# NOKTA TESTI  —  simulasyon yok, MAVLink yok, sadece liste dogrulama
# ============================================================================

from utils.lemniscate import lemniscate_noktalari                     # utils klasorundeki lemniscate.py dosyasindan sadece o fonksiyonu al

noktalar = lemniscate_noktalari()                                     # Hicbir parametre vermiyoruz -> dosyadaki varsayilan 8 deger kullanilir

print("toplam nokta:", len(noktalar))                                 # len() = listede kac eleman var; 240 bekliyoruz

for kuzey, dogu, asagi in noktalar:                                   # Liste 3'luk tuple'lardan olusuyor; her turda ucunu ayri isimlere acar
    print(round(kuzey, 1), round(dogu, 1), round(asagi, 1))           # round(sayi, 1) = 1 ondalik basamaga yuvarla, ekran okunabilir kalsin

# ============================================================================
# KÜTÜPHANELER
# ============================================================================

import math                                                           # Trigonometri (cos, sin) ve pi buradan gelir; Python ile hazır kurulu


# ============================================================================
# LEMNISCATE NOKTA ÜRETİCİ  —  saf matematik, MAVLink yok
# ============================================================================

def lemniscate_noktalari(                                             # Tarifi tanımlar; çağırmadıkça hiçbir satırı çalışmaz
    merkez_kuzey=35.0,                                                # Şeklin merkezi, kuzey ekseninde (metre)
    yari_uzunluk=25.0,                                                # Uzun eksende merkezden uca mesafe -> kuzey 10..60 m
    yari_genislik=12.5,                                               # Kısa eksende maksimum sapma -> doğu -12.5..+12.5 m
    irtifa_m=15.0,                                                    # Pozitif yükseklik (metre); aşağıda NED'e çevrilecek
    nokta_sayisi=240,                                                 # Sürekli eğri kaç parçaya bölünecek (az olursa şekil köşelenir)
    tur_sayisi=2,                                                     # Şekil kaç kez çizilecek
    baslangic_fazi=0.0,                                               # Eğrinin neresinden başlanacağı (radyan)
    yon=1,                                                            # 1 veya -1; -1 şekli aynalar (başka sayı sessizce şekli gerer)
):                                                                    # Parametre listesi bitti, gövde başlıyor

    noktalar = []                                                     # Sonuçları biriktireceğimiz boş kap; döngüden ÖNCE açılmalı
    toplam_aci = 2 * math.pi * tur_sayisi                             # Şeklin kapanması için t'nin kat edeceği toplam yol (2 tur = 4pi)

    for i in range(nokta_sayisi):                                     # 0'dan 239'a; 240 dahil değil, çünkü t=0 ile t=4pi aynı noktadır
        t = baslangic_fazi + toplam_aci * i / nokta_sayisi            # i/nokta_sayisi 0->1 oranı üretir, toplam_aci ile gerçek aralığa yayılır (radyan)

        kuzey = merkez_kuzey + yari_uzunluk * math.cos(t)             # cos(t) -1..+1 salınır, yarı uzunlukla çarpılıp merkeze eklenir
        dogu = yon * yari_genislik * math.sin(2 * t)                  # 2*t: kuzey bir tur atarken doğu iki tur atar -> ortadaki çaprazlama (sin(t) yazsan daire olur)
        asagi = -irtifa_m                                             # NED'de z aşağıyı gösterir; eksi işaretini tek yerde koyuyoruz

        noktalar.append((kuzey, dogu, asagi))                         # Üç sayıyı tuple olarak paketleyip listeye ekle (içteki parantez şart)

    return noktalar                                                   # Girinti for'un DIŞINDA; içeride olsa tek elemanlı liste dönerdi

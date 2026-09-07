import math

DUNYA_YARICAPI = 6378137.0


def ned_to_global(ev_enlem, ev_boylam, kuzey_m, dogu_m):
    """NED metre -> enlem/boylam derece. Duz-dunya yaklasimi."""
    enlem = ev_enlem + math.degrees(kuzey_m / DUNYA_YARICAPI)
    boylam = ev_boylam + math.degrees(
        dogu_m / (DUNYA_YARICAPI * math.cos(math.radians(ev_enlem)))
    )
    return enlem, boylam

# NOT: bu dosyayi SADECE AUTO yontemi kullanir.
# GUIDED metre calisir, cevrim gerekmez.

# NOT: bu dosyayi SADECE AUTO yontemi kullanir.
# GUIDED metre calisir, cevrim gerekmez.

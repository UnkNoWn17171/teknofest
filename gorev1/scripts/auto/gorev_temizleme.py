import sys
import time
from pymavlink import mavutil

ip = "tcp:127.0.0.1:5763"

captan = mavutil.mavlink_connection(ip)
captan.wait_heartbeat()
print("baglandi ->", captan.target_system, captan.target_component)


def gorev_sayisi():
    captan.mav.mission_request_list_send(
        captan.target_system,
        captan.target_component)

    msg = captan.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if msg is None:
        return None
    return msg.count


onceki = gorev_sayisi()
if onceki is None:
    print("MISSION_COUNT gelmedi - otopilot cevap vermiyor")
    sys.exit(1)
print("SILMEDEN ONCE madde sayisi:", onceki)

captan.mav.mission_clear_all_send(
    captan.target_system,
    captan.target_component)

ack = captan.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
if ack is None:
    print("MISSION_ACK gelmedi - silme onaylanmadi")
    sys.exit(1)
print("MISSION_ACK degeri:", ack.type, "(0 = kabul edildi)")

time.sleep(0.5)

sonraki = gorev_sayisi()
print("SILDIKTEN SONRA madde sayisi:", sonraki)

if sonraki is None:
    print("SONUC: dogrulanamadi, otopilot cevap vermedi")
    sys.exit(1)

if sonraki <= 1:
    print("SONUC: HAFIZA TEMIZ")
else:
    print("SONUC: TEMIZLENMEDI, hala", sonraki, "madde var")
    sys.exit(1)

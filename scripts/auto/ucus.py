import sys
import time
from pymavlink import mavutil

IP = "tcp:127.0.0.1:5763"
IRTIFA = 15.0
GPS_TIMEOUT = 120
IZLEME_TIMEOUT = 420

captan = mavutil.mavlink_connection(IP)
while True:
    captan.wait_heartbeat()
    if captan.target_system != 0:
        break
SYS = captan.target_system
COMP = captan.target_component
print("baglandi ->", SYS, COMP)

D = {"alt": None, "fix": None, "sat": None, "seq": None,
     "count": None, "mod": None, "armed": 0}


def oku(timeout=1):
    m = captan.recv_match(blocking=True, timeout=timeout)
    if m is None:
        return None
    t = m.get_type()
    if t == 'STATUSTEXT':
        print("  [OTOPILOT]", m.text)
    elif t == 'GLOBAL_POSITION_INT':
        D["alt"] = m.relative_alt / 1000.0
    elif t == 'GPS_RAW_INT':
        D["fix"] = m.fix_type
        D["sat"] = m.satellites_visible
    elif t == 'MISSION_CURRENT':
        D["seq"] = m.seq
    elif t == 'MISSION_COUNT':
        D["count"] = m.count
    elif t == 'HEARTBEAT':
        D["mod"] = captan.flightmode
        D["armed"] = captan.motors_armed()
    return m


def bekle(kosul, saniye, etiket):
    bitis = time.time() + saniye
    son = 0
    while time.time() < bitis:
        oku()
        if kosul():
            return True
        if time.time() - son >= 3:
            son = time.time()
            print("  ...", etiket, "| alt:", D["alt"], "fix:", D["fix"],
                  "mod:", D["mod"], "armed:", D["armed"])
    return False


# --- 0) VERI AKISI ---
captan.mav.request_data_stream_send(
    SYS, COMP, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
print("=== 0) VERI AKISI ISTENDI ===")
if not bekle(lambda: D["alt"] is not None, 10, "konum akisi"):
    print("HATA: GLOBAL_POSITION_INT gelmiyor")
    sys.exit(1)
print("VERI AKISI: tamam, irtifa ->", D["alt"])

# --- 1) GOREV KONTROL ---
captan.mav.mission_request_list_send(SYS, COMP)
if not bekle(lambda: D["count"] is not None, 5, "gorev sayimi"):
    print("HATA: MISSION_COUNT gelmedi")
    sys.exit(1)
print("GOREV KONTROL: hafizada", D["count"], "madde")
if D["count"] <= 1:
    print("HAFIZA BOS - once gorev_yukle.py calistir")
    sys.exit(1)
captan.mav.mission_ack_send(SYS, COMP, 0)
TOPLAM = D["count"]

# --- 2) GPS KILIDI ---
print("=== 2) GPS KILIDI ===")
if not bekle(lambda: D["fix"] is not None and D["fix"] >= 3,
             GPS_TIMEOUT, "gps"):
    print("HATA: 3D fix yok, fix =", D["fix"])
    sys.exit(1)
print("GPS: fix", D["fix"], "| uydu", D["sat"])

# --- 3) GUIDED ---
captan.mav.set_mode_send(
    SYS, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    captan.mode_mapping()["GUIDED"])
if not bekle(lambda: D["mod"] == "GUIDED", 10, "mod"):
    print("HATA: GUIDED olmadi ->", D["mod"])
    sys.exit(1)
print("MOD: GUIDED")

# --- 4) ARM ---
print("=== 4) ARM ===")
armlandi = False
bitis = time.time() + 60
while time.time() < bitis and not armlandi:
    captan.mav.command_long_send(
        SYS, COMP, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    bekle(lambda: D["armed"], 4, "arm")
    armlandi = bool(D["armed"])

if not armlandi:
    print("HATA: arm olmadi - yukaridaki [OTOPILOT] satirlarina bak")
    sys.exit(1)
print("ARM: tamam")

# --- 5) TAKEOFF ---
captan.mav.command_long_send(
    SYS, COMP, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, IRTIFA)
print("TAKEOFF verildi, hedef", IRTIFA, "m")

if not bekle(lambda: D["alt"] is not None and D["alt"] >= IRTIFA * 0.95,
             60, "tirmanis"):
    print("HATA: irtifaya ulasilmadi, son:", D["alt"])
    sys.exit(1)
print("IRTIFA: tamam ->", D["alt"])

# --- 6) AUTO ---
captan.mav.mission_set_current_send(SYS, COMP, 1)
time.sleep(0.5)
captan.mav.set_mode_send(
    SYS, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    captan.mode_mapping()["AUTO"])
if not bekle(lambda: D["mod"] == "AUTO", 10, "mod"):
    print("HATA: AUTO olmadi ->", D["mod"])
    sys.exit(1)
print("MOD: AUTO - gorev basladi")

# --- 7) IZLE ---
SON = TOPLAM - 1
bitis = time.time() + IZLEME_TIMEOUT
yazim = 0
while time.time() < bitis:
    oku()
    if D["seq"] is not None and D["seq"] >= SON:
        print("SON MADDEYE ULASILDI: seq", D["seq"])
        break
    if time.time() - yazim >= 1.0:
        yazim = time.time()
        print("madde", D["seq"], "| irtifa", D["alt"])

print("IZLEME BITTI - MAVProxy'de 'mode rtl' yaz")

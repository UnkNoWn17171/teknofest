import sys
import time
from pymavlink import mavutil

# ---------- AYARLAR ----------
IP = "tcp:127.0.0.1:5763"
IRTIFA = 15.0
HEDEF_KUZEY = 20.0      # NED metre, ev noktasina gore
HEDEF_DOGU = 0.0
IZLEME_SN = 40
# -----------------------------

# pozisyon maskesi: hiz/ivme/yaw alanlarini YOK SAY, sadece x,y,z'yi kullan
POZ_MASKE = 0b0000111111111000
FRAME = mavutil.mavlink.MAV_FRAME_LOCAL_NED

captan = mavutil.mavlink_connection(IP)
while True:
    captan.wait_heartbeat()
    if captan.target_system != 0:
        break
SYS = captan.target_system
COMP = captan.target_component
print("baglandi ->", SYS, COMP)

D = {"alt": None, "fix": None, "mod": None, "armed": 0,
     "x": None, "y": None, "z": None}


def oku(timeout=1):
    m = captan.recv_match(blocking=True, timeout=timeout)
    if m is None:
        return None
    t = m.get_type()
    if t == 'STATUSTEXT':
        print("  [OTOPILOT]", m.text)
    elif t == 'GLOBAL_POSITION_INT':
        D["alt"] = m.relative_alt / 1000.0
    elif t == 'LOCAL_POSITION_NED':
        D["x"] = m.x
        D["y"] = m.y
        D["z"] = m.z
    elif t == 'GPS_RAW_INT':
        D["fix"] = m.fix_type
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


def hedef_gonder(kuzey, dogu, asagi):
    captan.mav.set_position_target_local_ned_send(
        0,              # time_boot_ms, otopilot umursamiyor
        SYS, COMP,
        FRAME,
        POZ_MASKE,
        kuzey, dogu, asagi,     # pozisyon (m)
        0, 0, 0,                # hiz     -- maskede kapali
        0, 0, 0,                # ivme    -- maskede kapali
        0, 0)                   # yaw     -- maskede kapali


# --- 0) VERI AKISI ---
captan.mav.request_data_stream_send(
    SYS, COMP, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
print("=== 0) VERI AKISI ===")
if not bekle(lambda: D["alt"] is not None and D["x"] is not None,
             10, "akis"):
    print("HATA: konum mesajlari gelmiyor")
    sys.exit(1)
print("VERI AKISI: tamam | LOCAL_POSITION_NED:",
      round(D["x"], 1), round(D["y"], 1), round(D["z"], 1))

# --- 1) GPS ---
print("=== 1) GPS KILIDI ===")
if not bekle(lambda: D["fix"] is not None and D["fix"] >= 3, 120, "gps"):
    print("HATA: 3D fix yok")
    sys.exit(1)
print("GPS: fix", D["fix"])

# --- 2) GUIDED ---
captan.mav.set_mode_send(
    SYS, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    captan.mode_mapping()["GUIDED"])
if not bekle(lambda: D["mod"] == "GUIDED", 10, "mod"):
    print("HATA: GUIDED olmadi ->", D["mod"])
    sys.exit(1)
print("MOD: GUIDED")

# --- 3) ARM ---
print("=== 3) ARM ===")
bitis = time.time() + 60
while time.time() < bitis and not D["armed"]:
    captan.mav.command_long_send(
        SYS, COMP, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    bekle(lambda: D["armed"], 4, "arm")

if not D["armed"]:
    print("HATA: arm olmadi")
    sys.exit(1)
print("ARM: tamam")

# --- 4) TAKEOFF ---
captan.mav.command_long_send(
    SYS, COMP, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, IRTIFA)
print("TAKEOFF verildi, hedef", IRTIFA, "m")

if not bekle(lambda: D["alt"] is not None and D["alt"] >= IRTIFA * 0.95,
             60, "tirmanis"):
    print("HATA: irtifaya ulasilmadi, son:", D["alt"])
    sys.exit(1)
print("IRTIFA: tamam ->", D["alt"])
time.sleep(2)

# --- 5) TEK HEDEF ---
HEDEF_ASAGI = -IRTIFA
print("=== 5) TEK HEDEF:", HEDEF_KUZEY, HEDEF_DOGU, HEDEF_ASAGI, "===")
print("    (ACK gelmez -- dogrulama LOCAL_POSITION_NED ile)")

bitis = time.time() + IZLEME_SN
yazim = 0
en_yakin = 9999.0
while time.time() < bitis:
    hedef_gonder(HEDEF_KUZEY, HEDEF_DOGU, HEDEF_ASAGI)
    oku(0.2)

    if D["x"] is not None:
        fark = ((HEDEF_KUZEY - D["x"]) ** 2 +
                (HEDEF_DOGU - D["y"]) ** 2) ** 0.5
        en_yakin = min(en_yakin, fark)
        if time.time() - yazim >= 1.0:
            yazim = time.time()
            print("hedef", HEDEF_KUZEY, HEDEF_DOGU,
                  "| gercek", round(D["x"], 1), round(D["y"], 1),
                  round(D["z"], 1),
                  "| kalan", round(fark, 1), "m")

print("=== SONUC ===")
print("hedefe en cok yaklasma:", round(en_yakin, 2), "m")
if en_yakin < 2.0:
    print("MESAJ KABUL EDILDI - GUIDED calisiyor")
else:
    print("DRONE KIMILDAMADI - type_mask veya frame yanlis")

print("iniliyor...")
captan.mav.set_mode_send(
    SYS, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    captan.mode_mapping()["RTL"])

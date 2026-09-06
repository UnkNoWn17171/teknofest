import sys
import time
from pymavlink import mavutil

IP = "tcp:127.0.0.1:5763"

captan = mavutil.mavlink_connection(IP)
captan.wait_heartbeat()
SYS = captan.target_system
COMP = captan.target_component
print("baglandi ->", SYS, COMP)

durum = {"fix": None, "sat": None, "ekf": None, "mod": None}


def dinle(sure):
    bitis = time.time() + sure
    son = 0
    while time.time() < bitis:
        m = captan.recv_match(blocking=True, timeout=1)
        if m is None:
            continue
        t = m.get_type()
        if t == 'STATUSTEXT':
            print("  [OTOPILOT]", m.text)
        elif t == 'GPS_RAW_INT':
            durum["fix"] = m.fix_type
            durum["sat"] = m.satellites_visible
        elif t == 'EKF_STATUS_REPORT':
            durum["ekf"] = m.flags
        elif t == 'HEARTBEAT':
            durum["mod"] = captan.flightmode
        elif t == 'COMMAND_ACK':
            print("  [ACK] komut", m.command, "sonuc", m.result)
        if time.time() - son >= 3:
            son = time.time()
            print("  fix:", durum["fix"], "sat:", durum["sat"],
                  "ekf:", durum["ekf"], "mod:", durum["mod"],
                  "armed:", captan.motors_armed())


print("=== 1) 60 SANIYE IZLEME (GPS kilidi bekleniyor) ===")
dinle(60)

print("=== 2) GUIDED ===")
captan.mav.set_mode_send(
    SYS, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    captan.mode_mapping()["GUIDED"])
dinle(6)

print("=== 3) ARM DENEMESI ===")
captan.mav.command_long_send(
    SYS, COMP, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)
dinle(20)

print("=== SONUC ===")
print("son durum:", durum)
print("motorlar armli mi:", captan.motors_armed())

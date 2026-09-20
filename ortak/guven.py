# ============================================================================
# guven.py  --  script nasil biterse bitsin drone'u havada birakmaz
#
# NEDEN VAR:
#   Bu dronda GUID_TIMEOUT=3, FS_GCS_ENABLE=0, BATT_MONITOR=0.
#   Yani script olurse: hedef guncellenmez -> drone son noktada asili kalir ->
#   batarya izlenmedigi icin kimse uyarmaz -> batarya bitene kadar bekler.
#   Bu modul cikista her kosulda LAND (veya RTL) komutu gonderip DOGRULAR.
#
# SINIRI (dursutce soyluyorum):
#   atexit ve sinyaller SIGKILL'i, elektrik kesilmesini, laptopun uyumasini,
#   WiFi'nin kopmasini YAKALAYAMAZ. Bu bir yazilim agi, FS_GCS_ENABLE'in
#   yerine gecmez. Asil cozum o parametrenin acilmasi.
#
# kullanim:
#     import guven
#     guven.kur(captan, SYS, COMP)          # basta bir kez
#     ...
#     guven.iptal()                          # normal bitiste, en sonda
# ============================================================================

import atexit
import signal
import time
from pymavlink import mavutil

# ---------- AYARLAR ----------
CIKIS_MODU = "LAND"        # LAND: oldugu yere iner, ev noktasi gerektirmez
                           # RTL : eve doner AMA ev noktasi set edilmis olmali
DOGRULAMA_SURESI = 8       # mod degisimini kac saniye bekleyelim
DENEME = 3                 # acil durumda kac kez deneyelim
# -----------------------------

_durum = {"captan": None, "sys": 0, "comp": 1,
          "aktif": False, "calisiyor": False, "acil_calisti": False}

_sure = {"bitis": None}


# ---------------------------------------------------------------- yardimcilar

def _mesaj_bas(m):
    if m is not None and m.get_type() == 'STATUSTEXT':
        print("  [OTOPILOT]", m.text)


def mod_degistir(captan, sys_id, comp_id, ad, saniye=None):
    """Mod degistirir VE degistigini dogrular.

    Mod degisimi bir ISTEKTIR, emir degil; otopilot reddedebilir.
    Dogrulamazsan sonraki butun komutlarin sessizce cope gider."""
    if saniye is None:
        saniye = DOGRULAMA_SURESI

    harita = captan.mode_mapping() or {}
    if ad not in harita:
        print("  HATA: bilinmeyen mod ->", ad)
        return False

    captan.mav.set_mode_send(
        sys_id,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        harita[ad])

    bitis = time.time() + saniye
    while time.time() < bitis:
        m = captan.recv_match(blocking=True, timeout=1)
        if m is None:
            continue
        _mesaj_bas(m)
        if m.get_type() == 'HEARTBEAT' and m.get_srcSystem() == sys_id:
            if captan.flightmode == ad:
                return True
    return False


def arm_et(captan, sys_id, comp_id, saniye=30):
    """Arm eder ve arm oldugunu dogrular. Reddedilirse sebebini ekrana basar."""
    bitis = time.time() + saniye
    son_istek = 0.0
    while time.time() < bitis:
        if time.time() - son_istek >= 3:
            son_istek = time.time()
            captan.mav.command_long_send(
                sys_id, comp_id,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
        m = captan.recv_match(blocking=True, timeout=1)
        if m is None:
            continue
        _mesaj_bas(m)
        if m.get_type() == 'HEARTBEAT' and m.get_srcSystem() == sys_id:
            if captan.motors_armed():
                return True
    return False


def _armed_mi(captan, sys_id, saniye=3):
    """Arac su an armed mi? Emin olamazsak GUVENLI taraf: armed varsay."""
    bitis = time.time() + saniye
    while time.time() < bitis:
        m = captan.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if m is not None and m.get_srcSystem() == sys_id:
            return bool(captan.motors_armed())
    return True


# ---------------------------------------------------------------- guvenlik agi

def _acil():
    """Cikista calisir. Arac havadaysa CIKIS_MODU'na sokar ve dogrular."""
    if not _durum["aktif"] or _durum["calisiyor"]:
        return
    _durum["calisiyor"] = True
    _durum["acil_calisti"] = True      # finally blogu bunu gorup mod degistirmeyecek

    captan = _durum["captan"]
    sys_id = _durum["sys"]
    comp_id = _durum["comp"]

    print("\n" + "!" * 60)
    print("GUVENLIK AGI DEVREDE ->", CIKIS_MODU)

    if not _armed_mi(captan, sys_id):
        print("arac zaten disarm, birsey yapilmadi")
        print("!" * 60)
        _durum["aktif"] = False
        _durum["calisiyor"] = False
        return

    for i in range(1, DENEME + 1):
        if mod_degistir(captan, sys_id, comp_id, CIKIS_MODU):
            print("%s moduna gecildi (deneme %d)" % (CIKIS_MODU, i))
            print("!" * 60)
            _durum["aktif"] = False
            _durum["calisiyor"] = False
            return
        print("deneme %d basarisiz, tekrar deneniyor" % i)

    print("DIKKAT: %s MODUNA GECILEMEDI" % CIKIS_MODU)
    print("KUMANDADAN MUDAHALE ET -- drone havada olabilir")
    print("!" * 60)
    _durum["calisiyor"] = False


def _sinyal(imza, cerceve):
    """Ctrl+C ve kill icin. atexit zaten kayitli ama sinyalde once burasi caliisr."""
    print("\n[sinyal %d alindi]" % imza)
    _acil()
    raise SystemExit(1)


def kur(captan, sys_id, comp_id, cikis_modu=None):
    """Guvenlik agini kurar. Script basinda bir kez cagrilir."""
    global CIKIS_MODU
    if cikis_modu:
        CIKIS_MODU = cikis_modu

    _durum["captan"] = captan
    _durum["sys"] = sys_id
    _durum["comp"] = comp_id
    _durum["aktif"] = True
    _durum["acil_calisti"] = False

    atexit.register(_acil)
    signal.signal(signal.SIGINT, _sinyal)      # Ctrl+C
    signal.signal(signal.SIGTERM, _sinyal)     # kill
    print("guvenlik agi kuruldu -> cikista", CIKIS_MODU)


def acil_calisti():
    """Guvenlik agi devreye girdi mi? finally bloklari bunu KONTROL ETMELI,
    yoksa agin verdigi acil karari (LAND) ezip RTL yapabilirler."""
    return _durum["acil_calisti"]


def iptal():
    """Normal bitiste cagrilir. Inis zaten yapildiysa ag tekrar tetiklenmesin."""
    _durum["aktif"] = False
    print("guvenlik agi kapatildi (normal bitis)")


# ---------------------------------------------------------------- sure siniri

def sure_baslat(saniye):
    """Batarya izlenmedigi icin (BATT_MONITOR=0) tek korumamiz sure."""
    _sure["bitis"] = time.time() + saniye
    print("ucus sure siniri:", saniye, "sn")


def sure_doldu():
    return _sure["bitis"] is not None and time.time() > _sure["bitis"]


def kalan_sure():
    if _sure["bitis"] is None:
        return None
    return max(0.0, _sure["bitis"] - time.time())

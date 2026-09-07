import math
import os

# ================== 1) AYARLAR — degistirecegin tek yer ==================
BU_KLASOR = os.path.dirname(os.path.abspath(__file__))
CIKTI = os.path.join(BU_KLASOR, "gorev2_pist.sdf")

# Gazebo ENU: x = Dogu, y = Kuzey.  NED'e cevrim: NED_kuzey = gazebo_y
DIREKLER = [
    ("direk_1", 0.0, 20.0),
    ("direk_2", 0.0, 50.0),
]

# (isim, sekil, kenar_uzunlugu_m, renk, gazebo_x, gazebo_y)
HEDEFLER = [
    ("hedef_altigen",  "altigen", 2.0, "mavi",    -8.0, 42.0),   # BIZIM
    ("hedef_ucgen",    "ucgen",   1.0, "kirmizi",  7.0, 48.0),   # BIZIM
    ("celdirici_4x4",  "kare",    4.0, "mavi",     6.0, 33.0),   # sabit kanat
    ("celdirici_2x2",  "kare",    2.0, "kirmizi", -6.0, 55.0),   # sabit kanat
]

RENKLER = {
    "mavi":    "0 0 1 1",
    "kirmizi": "1 0 0 1",
    "turuncu": "1 0.5 0 1",
    "beyaz":   "1 1 1 1",
}

KALINLIK = 0.02      # hedeflerin yerden yuksekligi (m) — cok ince, ucus etkilenmez


# ================== 2) SEKIL URETICILER ==================
def altigen_noktalari(kenar):
    """Duzgun altigen. Kenar uzunlugu = merkeze uzaklik (altigenin ozelligi)."""
    r = kenar
    return [(r * math.cos(math.radians(60 * i)),
             r * math.sin(math.radians(60 * i))) for i in range(6)]


def ucgen_noktalari(kenar):
    """Eskenar ucgen. Merkeze uzaklik = kenar / kok(3)."""
    r = kenar / math.sqrt(3)
    return [(r * math.cos(math.radians(90 + 120 * i)),
             r * math.sin(math.radians(90 + 120 * i))) for i in range(3)]


def kare_noktalari(kenar):
    """Kare. Merkeze gore dort kose."""
    y = kenar / 2.0
    return [(-y, -y), (y, -y), (y, y), (-y, y)]


SEKIL_TABLOSU = {
    "altigen": altigen_noktalari,
    "ucgen":   ucgen_noktalari,
    "kare":    kare_noktalari,
}


# ================== 3) SDF YAZICILAR ==================
def hedef_sdf(isim, sekil, kenar, renk, x, y):
    """Bir hedefi yere serilmis duz poligon olarak SDF'e cevirir."""
    noktalar = SEKIL_TABLOSU[sekil](kenar)
    nokta_satirlari = "\n".join(
        f"            <point>{px:.4f} {py:.4f}</point>" for px, py in noktalar
    )
    r = RENKLER[renk]
    return f"""
    <model name="{isim}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <polyline>
{nokta_satirlari}
              <height>{KALINLIK}</height>
            </polyline>
          </geometry>
          <material>
            <ambient>{r}</ambient>
            <diffuse>{r}</diffuse>
            <specular>0 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def direk_sdf(isim, x, y):
    r = RENKLER["turuncu"]
    return f"""
    <model name="{isim}">
      <static>true</static>
      <pose>{x} {y} 1.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.2</radius><length>3.0</length></cylinder></geometry>
          <material><ambient>{r}</ambient><diffuse>{r}</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.2</radius><length>3.0</length></cylinder></geometry>
        </collision>
      </link>
    </model>"""


CIZGI_SDF = f"""
    <model name="baslangic_cizgisi">
      <static>true</static>
      <pose>0 0 0.02 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>24 0.6 0.02</size></box></geometry>
          <material><ambient>{RENKLER["beyaz"]}</ambient><diffuse>{RENKLER["beyaz"]}</diffuse></material>
        </visual>
      </link>
    </model>"""


# ================== 4) SABIT SABLON ==================
BAS = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="iris_runway">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"></plugin>

    <scene>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
      <sky></sky>
    </scene>

    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <attenuation>
        <range>1000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <include>
      <uri>model://runway</uri>
      <pose degrees="true">0 0 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://iris_with_gimbal</uri>
      <pose degrees="true">0 0 0.195 0 0 0</pose>
    </include>
"""

SON = """
  </world>
</sdf>
"""


# ================== 5) BIRLESTIR VE YAZ ==================
def uret():
    parcalar = [BAS]
    for isim, x, y in DIREKLER:
        parcalar.append(direk_sdf(isim, x, y))
    parcalar.append(CIZGI_SDF)
    for isim, sekil, kenar, renk, x, y in HEDEFLER:
        parcalar.append(hedef_sdf(isim, sekil, kenar, renk, x, y))
    parcalar.append(SON)

    with open(CIKTI, "w") as f:
        f.write("".join(parcalar))

    print(f"Yazildi: {CIKTI}")
    for isim, sekil, kenar, renk, x, y in HEDEFLER:
        print(f"  {isim:16s} {sekil:8s} kenar={kenar}m {renk:8s} "
              f"gazebo=({x},{y})  ->  NED=({y},{x})")


if __name__ == "__main__":
    uret()
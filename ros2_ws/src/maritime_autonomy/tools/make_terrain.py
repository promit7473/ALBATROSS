#!/usr/bin/env python3
"""Fetch ESRI World Imagery for the KML operating-area bbox, stitch it, and
generate a PX4-SITL-compatible Gazebo world (chattogram.sdf) with the real
satellite surface textured on the ground, georeferenced so the SITL GPS origin
is the bbox centre. Makes the sim look like the real Chattogram port.

Imagery: Esri World Imagery (attribution: Esri, Maxar, Earthstar Geographics).
Usage: python3 make_terrain.py [zoom]
"""
import importlib.util
import io
import math
import os
import sys
import time
import urllib.request

from PIL import Image

KML = '/home/mhpromit7473/Chattogram Port — Drone Operating Area & Exclusion Zones.kml'
WORLDS = '/home/mhpromit7473/PX4-Autopilot/Tools/simulation/gz/worlds'
TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
            "World_Imagery/MapServer/tile/{z}/{y}/{x}")
Z = int(sys.argv[1]) if len(sys.argv) > 1 else 16

_s = importlib.util.spec_from_file_location(
    'geo', '/home/mhpromit7473/maritime_ws/src/maritime_autonomy/maritime_autonomy/geo.py')
geo = importlib.util.module_from_spec(_s)
_s.loader.exec_module(geo)


def deg2num(lat, lon, z):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def main():
    m = geo.OperatingMap(KML)
    op = m.operating_polygon('Port Gate', '5km')
    lats = [p[0] for p in op.pts]
    lons = [p[1] for p in op.pts]
    lat_min, lat_max, lon_min, lon_max = min(lats), max(lats), min(lons), max(lons)
    clat, clon = (lat_min + lat_max) / 2, (lon_min + lon_max) / 2

    x0f, y0f = deg2num(lat_max, lon_min, Z)   # NW corner
    x1f, y1f = deg2num(lat_min, lon_max, Z)   # SE corner
    X0, X1 = int(math.floor(x0f)), int(math.floor(x1f))
    Y0, Y1 = int(math.floor(y0f)), int(math.floor(y1f))
    nx, ny = X1 - X0 + 1, Y1 - Y0 + 1
    print(f"bbox lat {lat_min:.5f}..{lat_max:.5f} lon {lon_min:.5f}..{lon_max:.5f}")
    print(f"fetching {nx}x{ny}={nx*ny} tiles @ z{Z} ...")

    canvas = Image.new('RGB', (nx * 256, ny * 256))
    fails = 0
    for xi in range(X0, X1 + 1):
        for yi in range(Y0, Y1 + 1):
            for attempt in range(3):
                try:
                    req = urllib.request.Request(
                        TILE_URL.format(z=Z, x=xi, y=yi),
                        headers={'User-Agent': 'maritime-sim/1.0'})
                    data = urllib.request.urlopen(req, timeout=20).read()
                    canvas.paste(Image.open(io.BytesIO(data)).convert('RGB'),
                                 ((xi - X0) * 256, (yi - Y0) * 256))
                    break
                except Exception as e:
                    if attempt == 2:
                        fails += 1
                        print("tile fail", xi, yi, e)
                    time.sleep(0.4)
    print(f"tiles done ({fails} failed)")

    def px(lat, lon):
        xf, yf = deg2num(lat, lon, Z)
        return (xf - X0) * 256, (yf - Y0) * 256
    l, t = px(lat_max, lon_min)
    r, b = px(lat_min, lon_max)
    img = canvas.crop((int(round(l)), int(round(t)), int(round(r)), int(round(b))))
    if img.width > 4096:
        img = img.resize((4096, int(4096 * img.height / img.width)))
    os.makedirs(WORLDS, exist_ok=True)
    png = os.path.join(WORLDS, 'chattogram_sat.png')
    img.save(png)
    print("saved", png, img.size)

    Wm = (lon_max - lon_min) * 111320.0 * math.cos(math.radians(clat))
    Hm = (lat_max - lat_min) * 111320.0
    print(f"plane {Wm:.0f} x {Hm:.0f} m, origin {clat:.6f},{clon:.6f}")

    world = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="chattogram">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene><grid>false</grid><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.82 0.95 1</background><shadows>true</shadows></scene>
    <light name="sunUTC" type="directional"><pose>0 0 500 0 0 0</pose><cast_shadows>true</cast_shadows><intensity>1</intensity><direction>0.3 0.4 -0.85</direction><diffuse>0.95 0.95 0.95 1</diffuse><specular>0.3 0.3 0.3 1</specular><attenuation><range>4000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation><spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot></light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry><surface><friction><ode/></friction></surface></collision>
      <visual name="satellite"><cast_shadows>false</cast_shadows>
        <geometry><plane><normal>0 0 1</normal><size>{Wm:.1f} {Hm:.1f}</size></plane></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.1 0.1 0.1 1</specular>
          <pbr><metal><albedo_map>{png}</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr></material></visual>
      <pose>0 0 0 0 0 0</pose></link><pose>0 0 0 0 0 0</pose></model>
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{clat:.8f}</latitude_deg><longitude_deg>{clon:.8f}</longitude_deg><elevation>0</elevation></spherical_coordinates>
  </world>
</sdf>'''
    sdf = os.path.join(WORLDS, 'chattogram.sdf')
    open(sdf, 'w').write(world)
    print("wrote", sdf)
    print(f"RUN:  make px4_sitl gz_x6  (PX4_GZ_WORLD=chattogram)")


if __name__ == '__main__':
    main()

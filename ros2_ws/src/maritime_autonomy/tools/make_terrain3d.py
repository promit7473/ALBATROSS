#!/usr/bin/env python3
"""Build a crisp 3D satellite-draped terrain MESH centred on a point (default the
Chattogram Port Gate / Dock 1), for PX4 Gazebo.
  - high-zoom Esri World Imagery (sharp up close)
  - real elevation from AWS Terrarium DEM
  - finer grid + per-vertex normals (well-lit, not dark)
Outputs OBJ+MTL+PNG mesh + PX4-SITL world chattogram.sdf (origin at the centre).

Usage: python3 make_terrain3d.py [half_m=1500] [sat_zoom=18] [clat] [clon]
"""
import io
import math
import os
import sys
import time
import urllib.request

import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None
WORLDS = '/home/mhpromit7473/PX4-Autopilot/Tools/simulation/gz/worlds'
OUT = os.path.join(WORLDS, 'chattogram_terrain')
SAT_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
           "World_Imagery/MapServer/tile/{z}/{y}/{x}")
DEM_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"

HALF = float(sys.argv[1]) if len(sys.argv) > 1 else 1500.0   # metres half-size
SAT_Z = int(sys.argv[2]) if len(sys.argv) > 2 else 18
CLAT = float(sys.argv[3]) if len(sys.argv) > 3 else 22.318725   # Dock 1 / Port Gate
CLON = float(sys.argv[4]) if len(sys.argv) > 4 else 91.813156
DEM_Z = 14
GX = GY = 320


def deg2num(lat, lon, z):
    n = 2 ** z
    return ((lon + 180) / 360 * n,
            (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n)


def mosaic(url, z, bbox):
    lat_min, lat_max, lon_min, lon_max = bbox
    X0 = int(math.floor(deg2num(lat_max, lon_min, z)[0]))
    Y0 = int(math.floor(deg2num(lat_max, lon_min, z)[1]))
    X1 = int(math.floor(deg2num(lat_min, lon_max, z)[0]))
    Y1 = int(math.floor(deg2num(lat_min, lon_max, z)[1]))
    nx, ny = X1 - X0 + 1, Y1 - Y0 + 1
    print(f"  z{z}: {nx}x{ny}={nx*ny} tiles")
    canvas = Image.new('RGB', (nx * 256, ny * 256))
    for xi in range(X0, X1 + 1):
        for yi in range(Y0, Y1 + 1):
            for _ in range(3):
                try:
                    req = urllib.request.Request(url.format(z=z, x=xi, y=yi),
                                                 headers={'User-Agent': 'maritime-sim/1.0'})
                    d = urllib.request.urlopen(req, timeout=20).read()
                    canvas.paste(Image.open(io.BytesIO(d)).convert('RGB'),
                                 ((xi - X0) * 256, (yi - Y0) * 256)); break
                except Exception:
                    time.sleep(0.3)
    l = (deg2num(lat_max, lon_min, z)[0] - X0) * 256
    t = (deg2num(lat_max, lon_min, z)[1] - Y0) * 256
    r = (deg2num(lat_min, lon_max, z)[0] - X0) * 256
    b = (deg2num(lat_min, lon_max, z)[1] - Y0) * 256
    return canvas.crop((int(round(l)), int(round(t)), int(round(r)), int(round(b))))


def main():
    os.makedirs(OUT, exist_ok=True)
    dlat = HALF / 111320.0
    dlon = HALF / (111320.0 * math.cos(math.radians(CLAT)))
    lat_min, lat_max = CLAT - dlat, CLAT + dlat
    lon_min, lon_max = CLON - dlon, CLON + dlon
    bbox = (lat_min, lat_max, lon_min, lon_max)
    print(f"centre {CLAT},{CLON}  half {HALF} m")

    satpath = os.path.join(OUT, 'sat.png')
    if os.path.exists(satpath) and '--refetch' not in sys.argv:
        print("satellite (cached)", Image.open(satpath).size)
    else:
        print("satellite ...")
        sat = mosaic(SAT_URL, SAT_Z, bbox)
        if sat.width > 8192:
            sat = sat.resize((8192, int(8192 * sat.height / sat.width)))
        sat.save(satpath); print("  texture", sat.size)

    print("DEM ...")
    dem = np.asarray(mosaic(DEM_URL, DEM_Z, bbox)).astype(np.float64)
    elev = dem[:, :, 0] * 256 + dem[:, :, 1] + dem[:, :, 2] / 256 - 32768
    elev = np.clip(elev, 0, None)
    dh, dw = elev.shape
    print(f"  DEM {dw}x{dh} elev {elev.min():.0f}..{elev.max():.0f} m")

    Wm = (lon_max - lon_min) * 111320.0 * math.cos(math.radians(CLAT))
    Hm = (lat_max - lat_min) * 111320.0

    # elevation grid Z[j][i]  (j=0 south -> north)
    Z = np.zeros((GY, GX))
    for j in range(GY):
        for i in range(GX):
            px = int(round(i / (GX - 1) * (dw - 1)))
            py = int(round((1 - j / (GY - 1)) * (dh - 1)))
            Z[j, i] = elev[py, px]
    Z = Z - Z[GY // 2, GX // 2]   # datum: terrain surface at spawn centre = z 0
    print(f"  terrain relative height {Z.min():.1f}..{Z.max():.1f} m about spawn")
    dx = Wm / (GX - 1); dy = Hm / (GY - 1)

    verts, uvs, norms = [], [], []
    for j in range(GY):
        for i in range(GX):
            x = (i / (GX - 1) - 0.5) * Wm
            y = (j / (GY - 1) - 0.5) * Hm
            verts.append((x, y, Z[j, i])); uvs.append((i / (GX - 1), j / (GY - 1)))
            zl = Z[j, max(i - 1, 0)]; zr = Z[j, min(i + 1, GX - 1)]
            zd = Z[max(j - 1, 0), i]; zu = Z[min(j + 1, GY - 1), i]
            nx = -(zr - zl) / (2 * dx); ny = -(zu - zd) / (2 * dy); nz = 1.0
            nl = math.sqrt(nx * nx + ny * ny + nz * nz)
            norms.append((nx / nl, ny / nl, nz / nl))

    def vid(i, j): return j * GX + i + 1
    faces = []
    for j in range(GY - 1):
        for i in range(GX - 1):
            a, b, c, d = vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)
            faces.append((a, b, c)); faces.append((a, c, d))

    obj = os.path.join(OUT, 'terrain.obj')
    with open(obj, 'w') as f:
        f.write("mtllib terrain.mtl\no terrain\n")
        for (x, y, z) in verts: f.write(f"v {x:.3f} {y:.3f} {z:.3f}\n")
        for (u, v) in uvs: f.write(f"vt {u:.5f} {v:.5f}\n")
        for (a, b, c) in norms: f.write(f"vn {a:.4f} {b:.4f} {c:.4f}\n")
        f.write("usemtl sat\n")
        for (a, b, c) in faces:
            f.write(f"f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}\n")
    with open(os.path.join(OUT, 'terrain.mtl'), 'w') as f:
        f.write("newmtl sat\nKa 0.7 0.7 0.7\nKd 1 1 1\nKs 0 0 0\nd 1\nillum 2\nmap_Kd sat.png\n")
    print(f"wrote {obj} ({len(verts)} v, {len(faces)} tris, {Wm:.0f}x{Hm:.0f} m)")

    world = f'''<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <world name="chattogram">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene><grid>false</grid><ambient>0.75 0.75 0.75 1</ambient><background>0.7 0.82 0.95 1</background><shadows>true</shadows></scene>
    <light name="sunUTC" type="directional"><pose>0 0 800 0 0 0</pose><cast_shadows>true</cast_shadows><intensity>1.2</intensity><direction>0.4 0.3 -0.85</direction><diffuse>1 1 1 1</diffuse><specular>0.2 0.2 0.2 1</specular><attenuation><range>6000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation><spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot></light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry><surface><friction><ode/></friction></surface></collision>
      <visual name="terrain"><cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>file://{OUT}/terrain.obj</uri></mesh></geometry></visual>
      <pose>0 0 0 0 0 0</pose></link><pose>0 0 0 0 0 0</pose></model>
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{CLAT:.8f}</latitude_deg><longitude_deg>{CLON:.8f}</longitude_deg><elevation>0</elevation></spherical_coordinates>
  </world>
</sdf>'''
    open(os.path.join(WORLDS, 'chattogram.sdf'), 'w').write(world)
    print("wrote chattogram.sdf (port-centred, normals, z%d)" % SAT_Z)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Super-realistic FLAT satellite terrain for the Chattogram Port KML area.

Native max-zoom (z19, ~0.28 m/px) Esri World Imagery over the whole operating
area, tiled into a grid of textured sub-planes (one <visual> plane each) so we
blow past the single-texture GPU limit and keep full resolution everywhere.
Ground stays a flat collision plane at z=0 -> zero takeoff/landing clipping.

Usage: python3 make_terrain_hires.py [zoom=19] [block_tiles=32] [workers=24]
Progress -> stdout. Resumable: cached tiles are skipped.
"""
import io, math, os, sys, time, ssl, urllib.request, importlib.util
from concurrent.futures import ThreadPoolExecutor, as_completed
from PIL import Image
Image.MAX_IMAGE_PIXELS = None

Z          = int(sys.argv[1]) if len(sys.argv) > 1 else 19
BLOCK      = int(sys.argv[2]) if len(sys.argv) > 2 else 32     # tiles per sub-plane side (32*256=8192px)
WORKERS    = int(sys.argv[3]) if len(sys.argv) > 3 else 24
WORLDS     = '/home/mhpromit7473/PX4-Autopilot/Tools/simulation/gz/worlds'
OUT        = os.path.join(WORLDS, 'chattogram_hires')
TILEDIR    = os.path.join(OUT, f'tiles_z{Z}')
KML        = '/home/mhpromit7473/Chattogram Port — Drone Operating Area & Exclusion Zones.kml'
TILE_URL   = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
              "World_Imagery/MapServer/tile/{z}/{y}/{x}")
CTX = ssl.create_default_context(); CTX.check_hostname = False; CTX.verify_mode = ssl.CERT_NONE

def d2n(lat, lon, z):
    n = 2**z
    return ((lon+180)/360*n, (1-math.asinh(math.tan(math.radians(lat)))/math.pi)/2*n)
def n2deg(x, y, z):
    n = 2**z
    return (math.degrees(math.atan(math.sinh(math.pi*(1-2*y/n)))), x/n*360-180)

def load_geo():
    s = importlib.util.spec_from_file_location(
        'geo', '/home/mhpromit7473/maritime_ws/src/maritime_autonomy/maritime_autonomy/geo.py')
    g = importlib.util.module_from_spec(s); s.loader.exec_module(g); return g

def tile_path(x, y): return os.path.join(TILEDIR, f'{x}_{y}.jpg')

def fetch_tile(x, y):
    p = tile_path(x, y)
    if os.path.exists(p) and os.path.getsize(p) > 0:
        return (x, y, True)
    url = TILE_URL.format(z=Z, x=x, y=y)
    for _ in range(4):
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'maritime-sim/1.0'})
            d = urllib.request.urlopen(req, timeout=20, context=CTX).read()
            if len(d) < 200:  # placeholder / empty
                time.sleep(0.3); continue
            with open(p, 'wb') as f: f.write(d)
            return (x, y, True)
        except Exception:
            time.sleep(0.4)
    return (x, y, False)

def main():
    os.makedirs(TILEDIR, exist_ok=True)
    # FULL KML extent (every coordinate) so the Bay of Bengal / river mouth in the
    # south are included -- not just the 5km Port Gate crop.
    import re
    txt = open(KML, encoding='utf-8', errors='replace').read()
    pts = re.findall(r'(-?\d+\.\d+),(-?\d+\.\d+)(?:,-?\d+\.?\d*)?', txt)
    lons = [float(a) for a, b in pts]; lats = [float(b) for a, b in pts]
    la0, la1, lo0, lo1 = min(lats), max(lats), min(lons), max(lons)
    # world origin = Port Gate (Dock 1) so the drone spawns on the port (land);
    # the sea sits to the south at negative-y offsets.
    clat, clon = 22.318725, 91.813156

    X0 = int(math.floor(d2n(la1, lo0, Z)[0])); X1 = int(math.floor(d2n(la0, lo1, Z)[0]))
    Y0 = int(math.floor(d2n(la1, lo0, Z)[1])); Y1 = int(math.floor(d2n(la0, lo1, Z)[1]))
    nx, ny = X1-X0+1, Y1-Y0+1
    total = nx*ny
    print(f"bbox {la0:.5f}..{la1:.5f}, {lo0:.5f}..{lo1:.5f}  origin ({clat:.6f},{clon:.6f})", flush=True)
    print(f"z{Z}: {nx}x{ny}={total} tiles -> {nx*256}x{ny*256}px", flush=True)

    # ---- fetch all tiles concurrently ----
    jobs = [(x, y) for x in range(X0, X1+1) for y in range(Y0, Y1+1)]
    done = 0; fail = 0; t0 = time.time()
    with ThreadPoolExecutor(max_workers=WORKERS) as ex:
        futs = [ex.submit(fetch_tile, x, y) for x, y in jobs]
        for fu in as_completed(futs):
            _, _, ok = fu.result(); done += 1
            if not ok: fail += 1
            if done % 500 == 0 or done == total:
                r = done/(time.time()-t0+1e-9)
                print(f"  tiles {done}/{total} ({fail} fail) {r:.0f}/s eta {int((total-done)/max(r,1))}s", flush=True)
    print(f"tiles done: {done-fail} ok, {fail} fail", flush=True)

    # ---- build sub-plane textures (blocks of BLOCK x BLOCK tiles) ----
    visuals = []
    bi = 0
    for xa in range(X0, X1+1, BLOCK):
        xb = min(xa+BLOCK-1, X1)
        for ya in range(Y0, Y1+1, BLOCK):
            yb = min(ya+BLOCK-1, Y1)
            bw, bh = (xb-xa+1)*256, (yb-ya+1)*256
            canvas = Image.new('RGB', (bw, bh), (60, 90, 120))
            for x in range(xa, xb+1):
                for y in range(ya, yb+1):
                    p = tile_path(x, y)
                    if os.path.exists(p) and os.path.getsize(p) > 0:
                        try: canvas.paste(Image.open(p).convert('RGB'), ((x-xa)*256, (y-ya)*256))
                        except Exception: pass
            texname = f'block_{xa}_{ya}.png'
            canvas.save(os.path.join(OUT, texname))
            # geo extent of this block (tile boundaries are exact)
            nlat, wlon = n2deg(xa, ya, Z)          # NW corner of block
            slat, elon = n2deg(xb+1, yb+1, Z)      # SE corner (next tile boundary)
            bclat, bclon = (nlat+slat)/2, (wlon+elon)/2
            ex_m = (bclon-clon)*111320.0*math.cos(math.radians(clat))   # East  -> world +x
            ny_m = (bclat-clat)*111320.0                                # North -> world +y
            Wm = (elon-wlon)*111320.0*math.cos(math.radians(clat))
            Hm = (nlat-slat)*111320.0
            visuals.append((f'sat_{bi}', texname, Wm, Hm, ex_m, ny_m))
            bi += 1
    print(f"built {len(visuals)} sub-plane textures", flush=True)

    vis_xml = "\n".join(
        f'''      <visual name="{name}"><cast_shadows>false</cast_shadows>
        <pose>{ex:.3f} {ny:.3f} 0 0 0 0</pose>
        <geometry><plane><normal>0 0 1</normal><size>{Wm:.2f} {Hm:.2f}</size></plane></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.1 0.1 0.1 1</specular>
          <pbr><metal><albedo_map>{os.path.join(OUT, tex)}</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr></material></visual>'''
        for (name, tex, Wm, Hm, ex, ny) in visuals)

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
{vis_xml}
      <pose>0 0 0 0 0 0</pose></link><pose>0 0 0 0 0 0</pose></model>
    <spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{clat:.8f}</latitude_deg><longitude_deg>{clon:.8f}</longitude_deg><elevation>0</elevation></spherical_coordinates>
  </world>
</sdf>'''
    open(os.path.join(WORLDS, 'chattogram.sdf'), 'w').write(world)
    print(f"WROTE {os.path.join(WORLDS, 'chattogram.sdf')} with {len(visuals)} hi-res sub-planes", flush=True)

if __name__ == '__main__':
    main()

"""Operating-area / geofence utilities for the maritime mission.

Loads a KML (e.g. the Chattogram Port operating-area + exclusion-zones file)
into structured zones and provides geofence / exclusion checks.

Design principle (per user): the KML drives *real* operations, but the autonomy
is coordinate-agnostic -- you can also fly anywhere by supplying manual
lat/lon, or by loading a different KML. Nothing here is hard-coded to Chattogram.

Run directly for a summary:
    python3 geo.py "/home/mhpromit7473/Chattogram Port ... .kml"
"""
import math
import sys
import xml.etree.ElementTree as ET

KML_NS = {'k': 'http://www.opengis.net/kml/2.2'}
_PM = '{http://www.opengis.net/kml/2.2}Placemark'

# name substrings that identify no-fly polygons in the Chattogram KML
_EXCLUSION_KEYS = ('restricted', 'runway', 'navy', 'naval', 'jett', 'berth',
                   'infrastructure', 'airport')

_JUNK_SUFFIXES = ('panel', 'segment', 'fragment')


def _coords(el):
    """Return [(lat, lon), ...] from a KML geometry element."""
    c = el.find('.//k:coordinates', KML_NS)
    if c is None or not c.text:
        return []
    out = []
    for tok in c.text.split():
        p = tok.split(',')
        if len(p) >= 2:
            out.append((float(p[1]), float(p[0])))  # KML is lon,lat -> (lat,lon)
    return out


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def point_in_polygon(lat, lon, poly):
    """Ray-casting test. poly = [(lat, lon), ...]."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        lat_i, lon_i = poly[i]
        lat_j, lon_j = poly[j]
        if ((lon_i > lon) != (lon_j > lon)) and \
           (lat < (lat_j - lat_i) * (lon - lon_i) / (lon_j - lon_i) + lat_i):
            inside = not inside
        j = i
    return inside


class Zone:
    def __init__(self, name, kind, pts):
        self.name = name
        self.kind = kind          # 'point' | 'polygon' | 'line'
        self.pts = pts            # [(lat, lon), ...]

    def centroid(self):
        la = sum(p[0] for p in self.pts) / len(self.pts)
        lo = sum(p[1] for p in self.pts) / len(self.pts)
        return la, lo

    def __repr__(self):
        la, lo = self.centroid()
        return f'<Zone {self.name!r} {self.kind} n={len(self.pts)} @({la:.5f},{lo:.5f})>'


class OperatingMap:
    """Structured view of a KML operating map."""

    def __init__(self, kml_path):
        self.zones = []
        _seen = set()
        root = ET.parse(kml_path).getroot()
        for pm in root.iter(_PM):
            nm = pm.find('k:name', KML_NS)
            nm = nm.text if nm is not None else '?'
            if pm.find('.//k:Point', KML_NS) is not None:
                kind = 'point'
            elif pm.find('.//k:Polygon', KML_NS) is not None:
                kind = 'polygon'
            elif pm.find('.//k:LineString', KML_NS) is not None:
                kind = 'line'
            else:
                continue
            pts = _coords(pm)
            if not pts:
                continue
            # drop the KML's decorative "apex" vertex markers (Points at 0,0)
            if 'apex' in nm.lower():
                continue
            if kind == 'point' and abs(pts[0][0]) < 1e-6 and abs(pts[0][1]) < 1e-6:
                continue
            # drop triangulation / rendering fragments ("... panel", "... segment")
            nm_lower = nm.lower()
            if any(nm_lower.endswith(s) for s in _JUNK_SUFFIXES):
                continue
            # deduplicate: same (kind, first-point) already seen -> skip
            key = (kind, pts[0])
            if key in _seen:
                continue
            _seen.add(key)
            self.zones.append(Zone(nm, kind, pts))

    def find(self, substr):
        s = substr.lower()
        return [z for z in self.zones if s in z.name.lower()]

    def dock(self, substr='Port Gate'):
        """Return (lat, lon) of the first matching dock Point."""
        for z in self.zones:
            if z.kind == 'point' and substr.lower() in z.name.lower():
                return z.pts[0]
        return None

    def operating_polygon(self, dock_substr='Port Gate', radius='5km'):
        """Return the operating-area polygon (e.g. a dock's 5km radius)."""
        key = radius.lower().replace(' ', '')
        for z in self.zones:
            n = z.name.lower().replace(' ', '')
            if z.kind == 'polygon' and dock_substr.lower().replace(' ', '') in n and key in n:
                return z
        return None

    def exclusions(self):
        """No-fly polygons (airport/navy/jetties/berths), excluding dock radii."""
        return [z for z in self.zones
                if z.kind == 'polygon'
                and any(k in z.name.lower() for k in _EXCLUSION_KEYS)
                and 'radius' not in z.name.lower()]

    def geofence_ok(self, lat, lon, op_poly=None):
        """True if inside operating area (if given) and outside all exclusions."""
        if op_poly is not None and not point_in_polygon(lat, lon, op_poly.pts):
            return False
        for z in self.exclusions():
            if point_in_polygon(lat, lon, z.pts):
                return False
        return True


def _summary(path):
    m = OperatingMap(path)
    print(f'loaded {len(m.zones)} zones from {path}\n')
    print('DOCK / BASE POINTS:')
    for z in m.zones:
        if z.kind == 'point':
            print(f'  {z.name:45} ({z.pts[0][0]:.5f}, {z.pts[0][1]:.5f})')
    print('\nEXCLUSION (no-fly) POLYGONS:')
    for z in m.exclusions():
        la, lo = z.centroid()
        print(f'  {z.name:45} centroid=({la:.5f},{lo:.5f})')
    op = m.operating_polygon('Port Gate', '5km')
    print(f'\nOperating polygon (Dock1 5km): {op}')
    if op:
        la, lo = op.centroid()
        # quick self-test
        print(f'  centroid inside operating & clear of exclusions: '
              f'{m.geofence_ok(la, lo, op)}')


if __name__ == '__main__':
    _summary(sys.argv[1] if len(sys.argv) > 1 else
             '/home/mhpromit7473/Chattogram Port — Drone Operating Area & Exclusion Zones.kml')

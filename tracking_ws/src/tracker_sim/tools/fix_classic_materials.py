#!/usr/bin/env python3
"""Convert Gazebo-classic OGRE material scripts in model.sdf files to PBR materials.

gz-sim's ogre2 renderer ignores <material><script>, so classic models that rely
on it render black. For every such material this looks up the named material in
the model's materials/scripts/*.material, takes its texture and rewrites the
block as <pbr><metal><albedo_map>. Idempotent: converted files are left alone.

Usage: fix_classic_materials.py <models_dir>
"""
import glob
import os
import re
import sys

MATERIAL_BLOCK = re.compile(r'<material>(?:(?!</material>).)*?<script>.*?</material>', re.S)


def script_textures(model_dir):
    """{material name: first texture file} from the model's .material scripts."""
    out = {}
    for path in glob.glob(os.path.join(model_dir, 'materials', 'scripts', '*.material')):
        with open(path) as f:
            text = f.read()
        for m in re.finditer(r'^\s*material\s+(\S+)', text, re.M):
            body = text[m.end():]
            nxt = re.search(r'^\s*(material|vertex_program|fragment_program)\s', body, re.M)
            body = body[:nxt.start()] if nxt else body
            tex = re.search(r'^\s*texture\s+(\S+)', body, re.M)
            if tex:
                out[m.group(1)] = tex.group(1)
    return out


def convert(model_dir):
    sdf_path = os.path.join(model_dir, 'model.sdf')
    if not os.path.exists(sdf_path):
        return 0
    with open(sdf_path) as f:
        sdf = f.read()
    textures = script_textures(model_dir)
    model = os.path.basename(model_dir)
    n = 0

    def repl(m):
        nonlocal n
        block = m.group(0)
        name = re.search(r'<name>([^<]+)</name>', block)
        tex = textures.get(name.group(1).strip()) if name else None
        if not tex:
            return ''                              # fall back to the mesh's own material
        normal = re.search(r'<normal_map>([^<]+)</normal_map>', block)
        normal_xml = (f'<normal_map>model://{model}/materials/textures/{normal.group(1).strip()}</normal_map>'
                      if normal else '')
        n += 1
        return ('<material><diffuse>1 1 1 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
                f'<pbr><metal><albedo_map>model://{model}/materials/textures/{tex}</albedo_map>'
                f'{normal_xml}<roughness>0.9</roughness><metalness>0.0</metalness></metal></pbr></material>')

    new = MATERIAL_BLOCK.sub(repl, sdf)
    if new != sdf:
        with open(sdf_path, 'w') as f:
            f.write(new)
    return n


def main():
    root = sys.argv[1]
    for d in sorted(glob.glob(os.path.join(root, '*'))):
        n = convert(d)
        if n:
            print(f'[materials] {os.path.basename(d)}: {n} script material(s) -> PBR')


if __name__ == '__main__':
    main()

#!/usr/bin/env bash
# Download the third-party assets the simulation needs (kept out of git):
#   assets/models/   Gazebo models: buildings, cars, street props (osrf/gazebo_models)
#   assets/models/people/  animated human meshes, recoloured into variants
#   assets/weights/  YOLO detector weights
# Then (re)generate the city world from src/tracker_sim/config/city.yaml.
# Safe to re-run: anything already present is skipped.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSETS="$WS/assets"
MODELS="$ASSETS/models"
WEIGHTS="$ASSETS/weights"
mkdir -p "$MODELS" "$WEIGHTS"

# Models used by the city world (must match what config/city.yaml references).
GAZEBO_MODELS=(
  apartment house_1 house_2 house_3 office_building gas_station fast_food
  grocery_store police_station post_office law_office salon thrift_shop
  school fire_station
  hatchback hatchback_blue hatchback_red suv pickup prius_hybrid
  ambulance fire_truck
  oak_tree pine_tree lamp_post fire_hydrant stop_sign bus_stop mailbox
  dumpster jersey_barrier construction_cone construction_barrel
  speed_limit_sign postbox fountain person_standing
)

missing=()
for m in "${GAZEBO_MODELS[@]}"; do [ -d "$MODELS/$m" ] || missing+=("$m"); done
if [ ${#missing[@]} -gt 0 ]; then
  echo "[assets] fetching ${#missing[@]} models from github.com/osrf/gazebo_models"
  tmp="$(mktemp -d)"
  trap 'rm -rf "$tmp"' EXIT
  git clone -q --depth 1 --filter=blob:none --sparse \
    https://github.com/osrf/gazebo_models.git "$tmp/gazebo_models"
  git -C "$tmp/gazebo_models" sparse-checkout set --no-cone "${missing[@]/#//}"
  for m in "${missing[@]}"; do cp -r "$tmp/gazebo_models/$m" "$MODELS/"; done
fi

# These (Gazebo-classic era) meshes reference textures by bare file name, which
# gz-sim resolves next to the mesh; link materials/textures/* into meshes/.
for m in "${GAZEBO_MODELS[@]}"; do
  [ -d "$MODELS/$m/materials/textures" ] && [ -d "$MODELS/$m/meshes" ] || continue
  for t in "$MODELS/$m/materials/textures/"*; do
    [ -e "$MODELS/$m/meshes/$(basename "$t")" ] || ln -s "../materials/textures/$(basename "$t")" "$MODELS/$m/meshes/"
  done
done

# Animated humans (skinned COLLADA from the Gazebo media set).
PEOPLE_SRC="$MODELS/people/source"
mkdir -p "$PEOPLE_SRC"
for anim in walk run stand talk_b; do
  f="$PEOPLE_SRC/$anim.dae"
  [ -s "$f" ] || { echo "[assets] fetching $anim.dae"
    curl -fsSL -o "$f" \
      "https://raw.githubusercontent.com/gazebosim/gazebo-classic/gazebo11/media/models/$anim.dae"; }
done

# YOLO weights (Ultralytics release assets).
for w in yolo11n yolo11s; do
  f="$WEIGHTS/$w.pt"
  [ -s "$f" ] || { echo "[assets] fetching $w.pt"
    curl -fsSL -o "$f" \
      "https://github.com/ultralytics/assets/releases/download/v8.3.0/$w.pt"; }
done

# Classic OGRE material scripts render black under ogre2 -> rewrite them as PBR.
python3 "$WS/src/tracker_sim/tools/fix_classic_materials.py" "$MODELS"

echo "[assets] building world"
python3 "$WS/src/tracker_sim/tools/build_world.py" \
  --config "$WS/src/tracker_sim/config/city.yaml" \
  --assets "$ASSETS" \
  --out "$WS/src/tracker_sim/worlds/city.sdf"

echo "[assets] done -> $ASSETS"

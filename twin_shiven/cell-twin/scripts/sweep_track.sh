#!/bin/zsh
# usage: scripts/sweep_track.sh <clearance_mm> <leadin_mm> <leadin_deg> [n_episodes]
cd ~/Professional/cell-twin
export TWIN_CLEARANCE_MM=$1 TWIN_LEADIN_MM=$2 TWIN_LEADIN_DEG=$3 TWIN_FRICTION=${TWIN_FRICTION:-0.4}
N=${4:-16}
seated=0; total=0
for ep in $(.venv/bin/python -c "
import json,glob
for p in sorted(glob.glob('inputs/nuc/episodes/objects/*/source.json')):
    m=json.load(open(p))
    if m.get('starting_pose')=='552d4ea6-1f26-4783-8244-c8ba3eef00fa' and not m['title'].startswith('Correction'): print(p.split('/')[-2])" | head -$N); do
  line=$(.venv/bin/python scripts/replay.py $ep --mode track --free-plug ${PER_EPISODE:-} 2>/dev/null | grep "^SEAT")
  total=$((total+1)); [[ "$line" == "SEAT: yes"* ]] && seated=$((seated+1))
  echo "clr=$1 lead=$2/$3 $ep ${line}"
done
echo "RESULT clr=$1 lead=$2mm/$3deg mu=$TWIN_FRICTION seated $seated/$total"

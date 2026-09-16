"""Native KiCad library export for a saved-board footprint; no board writes."""
import json, sys
from pathlib import Path
import pcbnew
board_path, ref, out_dir, entry = sys.argv[1:]
board = pcbnew.LoadBoard(board_path)
found = [f for f in board.GetFootprints() if f.GetReference() == ref]
if len(found) != 1:
    raise RuntimeError('reference missing or ambiguous')
original = found[0]
name = str(original.GetFPID().GetLibItemName())
if not name or '/' in name or '\\' in name or name in ['.', '..']:
    raise RuntimeError('unsafe footprint entry name')
copy = pcbnew.FOOTPRINT(original)
if entry:
    copy.SetFPIDAsString(str(original.GetFPID().GetLibNickname()) + ":" + entry)
copy.SetOrientationDegrees(0)
copy.SetPosition(pcbnew.VECTOR2I(0, 0))
plugin = pcbnew.PCB_IO_KICAD_SEXPR()
plugin.FootprintSave(out_dir, copy)
output_name = entry or name
loaded = plugin.FootprintLoad(out_dir, output_name, True)
if loaded is None:
    raise RuntimeError('native footprint readback failed')
matched, conflicting = [], []
for footprint in board.GetFootprints():
    if (str(footprint.GetFPID().GetLibNickname()), str(footprint.GetFPID().GetLibItemName())) == (str(original.GetFPID().GetLibNickname()), name):
        (conflicting if footprint.FootprintNeedsUpdate(loaded) else matched).append(footprint.GetReference())
if ref not in matched:
    raise RuntimeError('native roundtrip changed selected footprint')
print(json.dumps({'name': output_name, 'matched_references': sorted(matched), 'conflicting_references': sorted(conflicting), 'candidate': str(Path(out_dir) / (output_name + '.kicad_mod'))}))

# pbm_slicer

STL → outline + hatch G-code for the stage (mm output, `SCALE=1`, laser = tool 0 via `M3 S<0..1>`).

```
pip install -r requirements.txt
python pbm_slicer.py part.stl --preview            # writes part.gcode and part_preview.png
python pbm_slicer.py part.stl --layer-axis A       # layers on the focus axis
python pbm_slicer.py --help                        # all settings
python verify_gcode.py part.gcode                  # counts and laser/feed checks from the file
python -m pytest tests -q                          # tests (pytest)
```

| Folder | Content |
|---|---|
| `examples/` | Example STLs, regenerated G-code, previews, `verification_stats.json` |
| `tests/` | pytest suite |

The FreeCAD milling study (Item 8) that this slicer replaces is in `../NEW_freecad_study/`.

Details: `../NEW_docs/slicer_writeup.md`.

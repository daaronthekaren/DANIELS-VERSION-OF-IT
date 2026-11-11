import json
import re
import sys


# -------------------verify-----------------
def analyse(path):
    stats = {"file": path, "lines": 0, "layers": 0, "layer_moves": 0, "g1_moves": 0, "write_moves": 0,
             "travel_moves": 0, "laser_on_cmds": 0, "laser_off_cmds": 0, "laser_on_during_layer_move": 0,
             "max_feed": 0.0, "min_feed": None, "layer_heights": [], "non_monotonic_layers": 0,
             "ends_laser_off": None, "has_scale": False, "decimals_max": 0}
    laser = False
    last_layer = None
    with open(path, encoding="utf-8") as handle:
        for raw in handle:
            stats["lines"] += 1
            line = raw.split(";", 1)[0].strip()
            if raw.startswith("; -------------------layer"):
                stats["layers"] += 1
            if not line:
                continue
            if line.startswith("SCALE"):
                stats["has_scale"] = True
            if line.startswith("M3") or line.startswith("M5"):
                s = re.search(r"S([-+\d.]+)", line)
                on = line.startswith("M3") and s is not None and float(s.group(1)) > 0
                if on:
                    stats["laser_on_cmds"] += 1
                else:
                    stats["laser_off_cmds"] += 1
                laser = on
                continue
            if line.startswith("G1 ") or line.startswith("G0 "):
                words = dict((m.group(1), m.group(2)) for m in re.finditer(r"([A-Z])([-+\d.]+)", line[3:]))
                for v in words.values():
                    if "." in v:
                        stats["decimals_max"] = max(stats["decimals_max"], len(v.split(".")[1]))
                if "F" in words:
                    f = float(words["F"])
                    stats["max_feed"] = max(stats["max_feed"], f)
                    stats["min_feed"] = f if stats["min_feed"] is None else min(stats["min_feed"], f)
                if ("Z" in words or "A" in words) and "X" not in words and "Y" not in words:
                    stats["layer_moves"] += 1
                    z = float(words.get("Z", words.get("A")))
                    if last_layer is not None and z <= last_layer:
                        stats["non_monotonic_layers"] += 1
                    last_layer = z
                    stats["layer_heights"].append(z)
                    if laser:
                        stats["laser_on_during_layer_move"] += 1
                    continue
                stats["g1_moves"] += 1
                if laser:
                    stats["write_moves"] += 1
                else:
                    stats["travel_moves"] += 1
    stats["ends_laser_off"] = not laser
    heights = stats.pop("layer_heights")
    stats["first_layer"] = heights[0] if heights else None
    stats["last_layer"] = heights[-1] if heights else None
    return stats


if __name__ == "__main__":
    print(json.dumps([analyse(p) for p in sys.argv[1:]], indent=1))

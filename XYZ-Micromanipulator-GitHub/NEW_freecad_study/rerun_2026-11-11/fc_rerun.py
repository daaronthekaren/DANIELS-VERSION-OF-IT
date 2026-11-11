import os
import shutil
import sys
import traceback

import FreeCAD

src = os.environ["FC_SRC"]
out = os.environ["FC_OUT"]
os.makedirs(out, exist_ok=True)
log = open(os.path.join(out, "rerun_log.txt"), "w", encoding="utf-8")


def say(*a):
    print(*a)
    log.write(" ".join(str(x) for x in a) + "\n")
    log.flush()


try:
    say("FreeCAD", ".".join(FreeCAD.Version()[:3]), "rev", FreeCAD.Version()[3] if len(FreeCAD.Version()) > 3 else "")
    from Path.Post.Processor import PostProcessorFactory

    for label, recompute in (("stored_paths", False), ("recomputed", True)):
        work = os.path.join(out, f"{label}.FCStd")
        shutil.copy(src, work)
        doc = FreeCAD.openDocument(work)
        job = doc.getObject("Job")
        say(label, "post processor:", job.PostProcessor, "args:", job.PostProcessorArgs)
        if recompute:
            for obj in doc.Objects:
                obj.touch()
            doc.recompute()
            bad = [o.Name for o in doc.Objects if "Invalid" in o.State or "Error" in o.State]
            say(label, "recompute invalid objects:", bad)
        target = os.path.join(out, f"{label}.gcode")
        job.PostProcessorOutputFile = target
        processor = PostProcessorFactory.get_post_processor(job, job.PostProcessor)
        data = processor.export()
        if not data:
            say(label, "export returned nothing")
        else:
            for part, gcode in data:
                if gcode is not None:
                    with open(target, "w", encoding="utf-8", newline="\n") as f:
                        f.write(gcode)
                    say(label, "wrote", target, "lines", len(gcode.splitlines()), "part", part)
        FreeCAD.closeDocument(doc.Name)
except Exception:
    say(traceback.format_exc())
finally:
    log.close()

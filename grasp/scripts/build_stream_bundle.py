#!/usr/bin/env python3
"""Print a base64 stdin bundle containing ServoNode and the live pick driver."""

import base64
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
servo = base64.b64encode((HERE / "servo_cup_edge_xy.py").read_bytes()).decode("ascii")
policy = base64.b64encode((HERE / "pick_cycle_policy.py").read_bytes()).decode("ascii")
cup_detector = base64.b64encode((HERE / "cup_rim_detector.py").read_bytes()).decode("ascii")
driver_name = sys.argv[1] if len(sys.argv) > 1 else "run_streamed_live_pick_cycle.py"
driver = (HERE / driver_name).read_text(encoding="utf-8")
loader = (
    "import base64,types,sys\n"
    "_p=types.ModuleType('pick_cycle_policy')\n"
    "_p.__file__='pick_cycle_policy.py'\n"
    "sys.modules['pick_cycle_policy']=_p\n"
    f"exec(compile(base64.b64decode('{policy}'),'pick_cycle_policy.py','exec'),_p.__dict__)\n"
    "_c=types.ModuleType('cup_rim_detector')\n"
    "_c.__file__='cup_rim_detector.py'\n"
    "sys.modules['cup_rim_detector']=_c\n"
    f"exec(compile(base64.b64decode('{cup_detector}'),'cup_rim_detector.py','exec'),_c.__dict__)\n"
    "_m=types.ModuleType('servo_cup_edge_xy')\n"
    "_m.__file__='servo_cup_edge_xy.py'\n"
    "sys.modules['servo_cup_edge_xy']=_m\n"
    f"exec(compile(base64.b64decode('{servo}'),'servo_cup_edge_xy.py','exec'),_m.__dict__)\n"
)
print(base64.b64encode((loader + driver).encode("utf-8")).decode("ascii"))

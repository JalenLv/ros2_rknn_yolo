import subprocess
import sys


def test_vendor_import_has_no_heavy_or_network_install_dependencies():
    code = """
import sys
from yolo_tracking.trackers import BYTETracker
assert BYTETracker.__name__ == "BYTETracker"
for forbidden in ("lap", "scipy", "torch", "ultralytics", "rknnlite"):
    assert forbidden not in sys.modules, (forbidden, sys.modules.get(forbidden))
"""
    subprocess.run([sys.executable, "-c", code], check=True)

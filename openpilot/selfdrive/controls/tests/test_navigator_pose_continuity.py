"""Isolate model/control imports from suites that later fork replay workers."""
import subprocess
import sys
from pathlib import Path


def test_pose_continuity_engagement_pipeline():
  subprocess.run([sys.executable, '-m', 'pytest', '-o', 'addopts=', '-q',
                  str(Path(__file__).with_name('navigator_pose_continuity.py'))], check=True, timeout=60)

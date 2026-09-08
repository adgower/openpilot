import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from tools.navigator_staging.prepare import materialize, validate_destination, verify_package


class PackageTests(unittest.TestCase):
  def test_rejects_active_and_shared_paths(self):
    for path in ('/data/openpilot', '/data/pythonpath', '/data/params', '/data/safe_staging/x'):
      with self.assertRaises(ValueError):
        validate_destination(Path(path), Path('/data/openpilot'))

  def test_rejects_symlink_to_active_checkout(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp); active = root/'active'; active.mkdir(); link = root/'link'; link.symlink_to(active)
      with self.assertRaises(ValueError):
        validate_destination(link, active)

  def test_corrupt_bundle_fails_before_checkout_creation(self):
    with tempfile.TemporaryDirectory() as tmp:
      pkg=Path(tmp); (pkg/'parent.bundle').write_bytes(b'corrupt')
      (pkg/'package.json').write_text(json.dumps({'repos':[{'path':'.','bundle':'parent.bundle','sha':'0'*40,'sha256':'0'*64}]}))
      with self.assertRaises(ValueError):verify_package(pkg)

  def test_bundle_roundtrip_and_existing_checkout_refusal(self):
    with tempfile.TemporaryDirectory() as tmp:
      root=Path(tmp); src=root/'src';src.mkdir()
      def git(*args):return subprocess.check_output(['git','-C',str(src),*args],text=True).strip()
      git('init','-q');git('config','user.name','Fixture');git('config','user.email','fixture@example.invalid')
      (src/'content.txt').write_text('frozen source');git('add','.');git('commit','-qm','fixture')
      sha=git('rev-parse','HEAD');pkg=root/'pkg';pkg.mkdir();git('bundle','create',str(pkg/'parent.bundle'),'HEAD')
      digest=hashlib.sha256((pkg/'parent.bundle').read_bytes()).hexdigest()
      (pkg/'package.json').write_text(json.dumps({'repos':[{'path':'.','bundle':'parent.bundle','sha':sha,'sha256':digest}]}))
      target=root/'candidate';materialize(pkg,target,root/'boot')
      self.assertEqual((target/'content.txt').read_text(),'frozen source')
      self.assertEqual(subprocess.check_output(['git','-C',str(target),'rev-parse','HEAD'],text=True).strip(),sha)
      with self.assertRaises(ValueError):materialize(pkg,target,root/'boot')

  def test_manifest_cannot_escape_package(self):
    with tempfile.TemporaryDirectory() as tmp:
      pkg=Path(tmp); (pkg/'package.json').write_text(json.dumps({'repos':[{'path':'../escape','bundle':'../escape','sha':'0'*40,'sha256':'0'*64}]}))
      with self.assertRaises(ValueError):verify_package(pkg)

class ScriptGuards(unittest.TestCase):
  def test_shell_syntax(self):
    for script in Path(__file__).parent.glob('*.sh'):
      subprocess.run(['bash', '-n', str(script)], check=True)

  def test_build_refuses_local_checkout_before_hardware_access(self):
    result = subprocess.run(['bash', str(Path(__file__).parent/'build_only.sh'), str(Path.cwd())], capture_output=True, text=True)
    self.assertEqual(result.returncode, 2)
    self.assertIn('Requires /data/navigator-staging/', result.stdout)

  def test_boot_refuses_requested_angle_before_hardware_access(self):
    result = subprocess.run(['bash', str(Path(__file__).parent/'launch_staging.sh'), 'requested'], capture_output=True, text=True)
    self.assertEqual(result.returncode, 2)
    self.assertIn('Only A2/shadow staging permitted', result.stdout)

if __name__=='__main__':unittest.main()

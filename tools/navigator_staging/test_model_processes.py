import unittest
from tools.navigator_staging.check_model_processes import is_model_command

class ModelProcessTests(unittest.TestCase):
  def test_real_model_processes(self):
    for argv in (['python3','/data/openpilot/openpilot/selfdrive/modeld/modeld.py'],
                 ['python3.12','-u','-m','openpilot.selfdrive.modeld.dmonitoringmodeld'],
                 ['openpilot.selfdrive.modeld.modeld'], ['dmonitoringmodeld'],
                 ['/data/openpilot/selfdrive/modeld/modeld.py']):
      self.assertTrue(is_model_command(argv), argv)

  def test_searches_and_builds_do_not_count(self):
    for argv in (['grep','-E','[m]odeld.py|[d]monitoringmodeld.py'],
                 ['bash','-c','ps | grep modeld.py'],
                 ['python3','-c','print("modeld.py")'],
                 ['python3','compile_modeld.py'],
                 ['bash','build_only.sh'], []):
      self.assertFalse(is_model_command(argv), argv)

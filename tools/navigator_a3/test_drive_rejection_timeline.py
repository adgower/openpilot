import unittest
from tools.navigator_a3.drive_rejection_timeline import decode_lmc2, prior_sample, merge_windows

class TestTimeline(unittest.TestCase):
  def test_recorded_neutral(self):
    d=decode_lmc2(bytes.fromhex('04147d0fa200801e'))
    self.assertEqual(d['mode'],0)
    self.assertAlmostEqual(d['curvature_inv_m'],0)
    self.assertAlmostEqual(d['path_angle_rad'],0)
  def test_recorded_mode_one(self):
    d=decode_lmc2(bytes.fromhex('14231d0fa2008004'))
    self.assertEqual(d['mode'],1)
    self.assertAlmostEqual(d['curvature_inv_m'],-0.01536)
    self.assertAlmostEqual(d['path_angle_rad'],0)
  def test_length(self):
    with self.assertRaises(ValueError): decode_lmc2(b'\x00')
  def test_no_future_or_stale_imputation(self):
    rows=[{'t_ns':10,'valid':True,'x':1},{'t_ns':20,'valid':False,'x':2}]
    self.assertIsNone(prior_sample(rows,9,10))
    self.assertEqual(prior_sample(rows,19,10)['x'],1)
    self.assertFalse(prior_sample(rows,20,10)['valid'])
    self.assertIsNone(prior_sample(rows,31,10))
  def test_merge_preserves_disjoint_windows(self):
    self.assertEqual(merge_windows([(20,40),(5,15),(10,21),(50,55)]),[(5,40),(50,55)])

if __name__=='__main__': unittest.main()

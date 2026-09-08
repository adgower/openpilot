"""Verify replay boundary semantics against fresh compiled pinned stock libraries."""
import ctypes
from pathlib import Path
import tempfile
import unittest
from tools.navigator_a3.compiled_recorded_admission import build,snapshot,agreement_eligible,classify_src

class CompiledRecordedAdmissionTest(unittest.TestCase):
  def test_invalid_envelope_and_unknown_tags_do_not_count_as_agreement(self):
    for src in (-1,0x88,0xBF,0xC8,0xFF):
      self.assertEqual(classify_src(src)['kind'],'unknown')
    self.assertEqual(classify_src(0x80)['kind'],'returned')
    self.assertEqual(classify_src(0xC0)['kind'],'rejected')
    self.assertFalse(agreement_eligible(False,{'provenance':{'message_valid':True}}))
    self.assertFalse(agreement_eligible(True,{'provenance':{'message_valid':False}}))
    self.assertFalse(agreement_eligible(True,None))
    self.assertTrue(agreement_eligible(True,{'provenance':{'message_valid':True}}))

  def test_rx_permission_stale_tick_and_instrumentation(self):
    with tempfile.TemporaryDirectory() as d:
      libs=[build(Path('/private/tmp/navigator-a3-opendbc'),Path(d),trace) for trace in (False,True)]
      for lib,_ in libs:lib.init(3,0)
      def packet(address,data,tx):
        results=[lib.packet(address,0,(ctypes.c_ubyte*len(data)).from_buffer_copy(bytes(data)),len(data),tx) for lib,_ in libs]
        self.assertEqual(results[0],results[1]);self.assertEqual(snapshot(libs[0][0]),snapshot(libs[1][0]));return results[0]
      # Fresh compiled state disallows active command; no permission seeding.
      active=[16,0,125,15,162,0,128,0] # neutral geometry, active mode
      self.assertEqual(packet(0x3d6,active,1),0)
      traced,meta=libs[1]
      errors=[meta['checks'][traced.failures(i)]['expression'] for i in range(512) if traced.failures(i)!=-1]
      self.assertIn('!controls_allowed && steer_control_enabled',errors)
      # Real cruise RX encoding supplies a rising engagement edge.
      packet(0x165,[0,4,0,0,0,0,0,0],0)
      self.assertTrue(snapshot(traced)['controls_allowed'])
      self.assertEqual(packet(0x3d6,active,1),1)
      # Final wire gate blocks nonneutral path angle under either active mode.
      for mode in (1,3):
        nonneutral=active.copy();nonneutral[0]=mode<<4;nonneutral[4]+=4
        self.assertEqual(packet(0x3d6,nonneutral,1),0)
        self.assertTrue(snapshot(traced)['controls_allowed'])
      # Missing required RX beyond one second clears permission on safety tick.
      for lib,_ in libs:lib.timer(2000001);lib.tick()
      self.assertEqual(snapshot(libs[0][0]),snapshot(traced))
      self.assertFalse(snapshot(traced)['controls_allowed'])
      self.assertTrue(snapshot(traced)['rx_checks_invalid'])
if __name__=='__main__':unittest.main()

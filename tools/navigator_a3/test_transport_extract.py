import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

from tools.navigator_a3 import transport_extract as transport


def message(kind, values, t=100, valid=True):
  return NS(which=lambda: kind, logMonoTime=t, valid=valid, **{kind: values})


def packet(src, data=b'\x01\x02', address=0x3d6):
  return NS(src=src, dat=data, address=address)


class TransportExtractionTest(unittest.TestCase):
  def test_route_and_numeric_segment(self):
    cases = {'/raw/2026-09-01--12-30-00--10/rlog.zst': ('2026-09-01--12-30-00', 10),
             '/Downloads/dongle_2026-09-01--12-30-00--2--rlog': ('dongle_2026-09-01--12-30-00', 2),
             '/raw/dongle_2026-09-01--12-30-00--2/rlog.bz2': ('dongle_2026-09-01--12-30-00', 2)}
    for path, expected in cases.items():
      self.assertEqual(transport.route_segment(Path(path)), expected)
    paths = [Path('/raw/r--10/rlog'), Path('/raw/r--2/rlog')]
    self.assertEqual(sorted(paths, key=transport.input_sort_key), paths[::-1])

  def test_keep_all_sendcan_and_only_tagged_can_with_identity(self):
    msg = message('can', [packet(0), packet(128), packet(193), packet(255)], valid=False)
    rows = transport.normalize_event(msg, 'route', '/rlog', 7)
    self.assertEqual([r['kind'] for r in rows], ['returned', 'rejected', 'unknown'])
    self.assertEqual([r['bus'] for r in rows], [0, 1, None])
    self.assertEqual([r['packet_index'] for r in rows], [1, 2, 3])
    self.assertEqual(rows[0]['data_hex'], '0102')
    self.assertEqual(rows[0]['dlc'], 2)
    self.assertEqual(rows[0]['t_ns'], 100)
    self.assertFalse(rows[0]['valid'])
    self.assertEqual(rows[0]['source_event_index'], 7)
    self.assertEqual(rows[0]['provenance'], 'recorded')
    rows = transport.normalize_event(message('sendcan', [packet(0), packet(255)]), 'r', 'f', 0)
    self.assertEqual([r['kind'] for r in rows], ['published', 'published'])
    self.assertEqual([r['bus'] for r in rows], [0, None])

  def test_health_preserves_panda_identity_and_native_values(self):
    panda = NS(safetyTxBlocked=12, controlsAllowed=False, safetyRxChecksInvalid=True,
               safetyModel='ford', safetyParam=4, alternativeExperience=0)
    rows = transport.normalize_event(message('pandaStates', [panda, panda]), 'r', 'f', 8)
    self.assertEqual([r['panda_index'] for r in rows], [0, 1])
    self.assertEqual(rows[0]['safetyTxBlocked'], 12)
    self.assertEqual(rows[0]['safetyModel'], 'ford')
    self.assertTrue(rows[0]['safetyRxChecksInvalid'])
    self.assertEqual(transport.normalize_event(message('other', []), 'r', 'f', 0), [])

  def test_extract_deduplicates_content_and_never_sorts_timestamps(self):
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory).resolve()
      paths = [root / n for n in ['route--10--rlog', 'route--2--rlog', 'route--11--rlog']]
      for path, data in zip(paths, [b'late', b'early', b'late'], strict=True):
        path.write_bytes(data)
      def reader(path):
        return [message('sendcan', [packet(0)], t=t) for t in (200, 100)]
      manifest = transport.extract(paths, root / 'out', reader_factory=reader)
      rows = [json.loads(line) for line in (root / 'out/transport-events.jsonl').read_text().splitlines()]
      self.assertEqual([r['order'] for r in rows], list(range(4)))
      self.assertEqual([r['t_ns'] for r in rows], [200, 100, 200, 100])
      self.assertEqual(Path(rows[0]['source_file']), paths[1])
      self.assertEqual(manifest['event_count'], 4)
      self.assertEqual(manifest['duplicates'][0]['path'], str(paths[2]))
      self.assertEqual(manifest['duplicates'][0]['duplicate_of'], str(paths[0]))
      self.assertEqual(len(manifest['events_sha256']), 64)
      self.assertEqual(len(manifest['inputs'][0]['sha256']), 64)


if __name__ == '__main__':
  unittest.main()

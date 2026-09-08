import unittest
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from tools.navigator_a3 import eps_identity as eps


def record(route='donor', fw=b'NL14-14D003-AE', request=None, **kw):
  return dict(route=route, file='rlog.zst', sha256='abc', message_index=10,
              logMonoTime=123456, valid=True, entry_index=0,
              entry=dict(address=0x730, responseAddress=0x738, bus=0, subAddress=0,
                         ecu='adas', brand='mazda', fwVersion=fw,
                         request=[b'\x22\xf1\x88'] if request is None else request, **kw))


class IdentityTest(unittest.TestCase):
  def test_generic_f188_and_full_provenance(self):
    out = eps.inventory([record()])
    item = out['routes']['donor']['candidates'][0]
    self.assertEqual(item['identifier']['did'], 'f188')
    self.assertEqual(item['identifier']['label'], 'diagnostic identifier')
    self.assertEqual(item['fwVersion_hex'], b'NL14-14D003-AE'.hex())
    self.assertEqual(item['request_hex'], ['22f188'])
    self.assertEqual(item['observations'][0]['logMonoTime'], 123456)
    self.assertEqual(item['brand'], 'mazda')

  def test_f110_prefix_only_removed_with_matching_request(self):
    raw = bytes.fromhex('f110') + b'DSNL14-3F964-AB' + bytes.fromhex('de000102')
    matched = eps.decode_identifier(raw, [b'\x22\xf1\x10'])
    self.assertEqual(matched['value_hex'], raw[2:].hex())
    self.assertEqual(matched['prefix_removed_hex'], 'f110')
    unproved = eps.decode_identifier(raw, [])
    self.assertEqual(unproved['value_hex'], raw.hex())
    inconsistent = eps.decode_identifier(raw, [b'\x22\xf1\x88'])
    self.assertEqual(inconsistent['value_hex'], raw.hex())
    self.assertIn('payload_request_did_mismatch', inconsistent['issues'])

  def test_repeated_observations_dedup_but_distinct_bus_retained(self):
    a, b, c = record(), record(), record()
    b['message_index'] = 11
    c['entry']['bus'] = 1
    items = eps.inventory([a, b, c])['routes']['donor']['candidates']
    self.assertEqual([x['observation_count'] for x in items], [2, 1])
    self.assertEqual([x['message_index'] for x in items[0]['observations']], [10, 11])

  def test_conflict_same_did_not_different_did(self):
    out = eps.inventory([record(), record(fw=b'other'), record(fw=b'other2', request=[b'\x22\xf1\x10'])])
    self.assertEqual(len(out['routes']['donor']['conflicts']), 1)
    self.assertEqual(out['routes']['donor']['conflicts'][0]['did'], 'f188')
    self.assertEqual(eps.inventory([record(), record(fw=b'other', request=[b'\x22\xf1\x10'])])['routes']['donor']['conflicts'], [])

  def test_routes_never_merge_and_missing_is_explicit(self):
    out = eps.inventory([record(), record(route='a2', fw=b'other')], routes=['empty'])
    self.assertEqual(len(out['routes']['donor']['candidates']), 1)
    self.assertEqual(len(out['routes']['a2']['candidates']), 1)
    self.assertEqual(out['routes']['empty']['status'], 'missing')
    self.assertEqual(out['comparisons'][0]['status'], 'different_observed_values')

  def test_missing_fields_and_invalid_records_retained_as_issues(self):
    a = record()
    a['entry'] = {'address': 0x730}
    b = record()
    b['valid'] = False
    c = record()
    c['entry']['fwVersion'] = 'not bytes'
    out = eps.inventory([a,b,c])['routes']['donor']
    self.assertEqual(len(out['candidates']), 3)
    self.assertIn('missing_fwVersion', out['candidates'][0]['issues'])
    self.assertIn('invalid_carParams_publication', out['candidates'][1]['issues'])
    self.assertIn('invalid_fwVersion_type', out['candidates'][2]['issues'])

  def test_ambiguous_request_cannot_classify_or_strip(self):
    raw = b'\xf1\x10data'
    decoded = eps.decode_identifier(raw, [b'\x22\xf1\x10', b'\x22\xf1\x88'])
    self.assertIsNone(decoded['did'])
    self.assertEqual(decoded['value_hex'], raw.hex())
    self.assertIn('multiple_requested_dids', decoded['issues'])

  def test_combination_retains_schema_and_route_provenance(self):
    a, b = eps.inventory([record()]), eps.inventory([record(route='a2')])
    a['schema'], b['schema'] = {'path': 'old', 'sha256': 'a'}, {'path': 'new', 'sha256': 'b'}
    out = eps.combine_inventories([a, b])
    self.assertEqual(out['comparisons'][0]['status'], 'same_observed_value')
    self.assertEqual(out['routes']['donor']['candidates'], a['routes']['donor']['candidates'])
    self.assertEqual([x['schema']['sha256'] for x in out['source_manifests']], ['a', 'b'])

  def test_empty_and_invalid_only_are_unusable(self):
    for a in [record(fw=b''), dict(record(), valid=False)]:
      out = eps.inventory([a])['routes']['donor']
      self.assertEqual(out['status'], 'unusable')
      self.assertEqual(out['usable_candidate_count'], 0)

  def test_cli_reads_compressed_publications_with_explicit_schema_import(self):
    import capnp
    import zstandard
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      imports = root / 'imports'
      imports.mkdir()
      (imports / 'fw.capnp').write_text("""@0xbd9652accec48b53;
struct Firmware {
  address @0 :UInt32; responseAddress @1 :UInt32; bus @2 :UInt8;
  subAddress @3 :UInt8; ecu @4 :Text; brand @5 :Text;
  fwVersion @6 :Data; request @7 :List(Data);
}
""")
      schema = root / 'log.capnp'
      schema.write_text("""@0xb822623967855f39;
using F = import "/fw.capnp";
struct Event {
  logMonoTime @0 :UInt64; valid @1 :Bool;
  union { carParams @2 :CarParams; other @3 :Void; }
}
struct CarParams { carFw @0 :List(F.Firmware); }
""")
      loaded = capnp.load(str(schema), imports=[str(imports)])
      msg = loaded.Event.new_message(logMonoTime=42, valid=True)
      params = msg.init('carParams')
      fw = params.init('carFw', 1)[0]
      fw.address, fw.responseAddress, fw.bus, fw.subAddress = 0x730, 0x738, 0, 0
      fw.ecu, fw.brand, fw.fwVersion, fw.request = 'adas', 'hyundai', b'\xf1\x10binary\x00\xff', [b'\x22\xf1\x10']
      path, output = root / 'rlog.zst', root / 'output.json'
      path.write_bytes(zstandard.ZstdCompressor().compress(msg.to_bytes()))
      subprocess.run([sys.executable, eps.__file__, '--schema', str(schema),
                      '--import-dir', str(imports), '--route', 'synthetic',
                      '--logs', str(path), '--output', str(output)], check=True, capture_output=True)
      result = json.loads(output.read_text())
      item = result['routes']['synthetic']['candidates'][0]
      self.assertEqual(item['fwVersion_hex'], 'f11062696e61727900ff')
      self.assertEqual(item['observations'][0]['message_index'], 0)
      self.assertEqual(item['observations'][0]['logMonoTime'], 42)
      self.assertEqual(result['inputs'][0]['sha256'], eps.file_sha256(path))
      self.assertIn(str(imports.resolve()), result['schema']['import_dirs'])

  def test_did_prefix_without_identifier_is_unusable(self):
    result = eps.inventory([record(fw=b'\xf1\x10', request=[b'\x22\xf1\x10'])])['routes']['donor']
    self.assertEqual(result['status'], 'unusable')
    item = result['candidates'][0]
    self.assertEqual(item['fwVersion_hex'], 'f110')
    self.assertIn('empty_decoded_identifier', item['identifier']['issues'])

  def test_only_literal_true_publications_are_usable(self):
    for valid in (None, 0, 1, '', 'true', False):
      with self.subTest(valid=valid):
        result = eps.inventory([dict(record(), valid=valid)])['routes']['donor']
        self.assertEqual(result['status'], 'unusable')
        self.assertIn('invalid_carParams_publication', result['candidates'][0]['issues'])
        self.assertIs(result['candidates'][0]['observations'][0]['valid'], valid)

  def test_unknown_routes_are_rejected_without_merging_evidence(self):
    records = [record(route=route) for route in ('', None, 0, '   ')]
    records.append(record())
    del records[-1]['route']
    result = eps.inventory(records)
    self.assertEqual(result['routes'], {})
    self.assertEqual(result['comparisons'], [])
    self.assertEqual(len(result['rejected_records']), 5)
    for rejected in result['rejected_records']:
      self.assertEqual(rejected['reason'], 'invalid_route_identity')
      self.assertEqual(rejected['record']['file'], 'rlog.zst')
      self.assertEqual(rejected['record']['entry']['fwVersion']['bytes_hex'], b'NL14-14D003-AE'.hex())
    json.dumps(result)

  def test_optional_scanner_flags_are_preserved_and_distinguish_candidates(self):
    a = record(logging=True, obdMultiplexing=False)
    b = record(logging=False, obdMultiplexing=True)
    items = eps.inventory([a, b])['routes']['donor']['candidates']
    self.assertEqual(len(items), 2)
    self.assertIs(items[0]['logging'], True)
    self.assertIs(items[0]['obdMultiplexing'], False)
    self.assertIs(items[1]['logging'], False)
    self.assertIs(items[1]['obdMultiplexing'], True)
    self.assertEqual(items[0]['issues'], [])

  def test_non_target_entries_not_selected(self):
    a = record()
    a['entry']['address'] = 0x7e0
    self.assertEqual(eps.inventory([a])['routes']['donor']['status'], 'missing')


if __name__ == '__main__':
  unittest.main()

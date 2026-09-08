"""Reconcile EPS metadata inventories with preserved manifests and render a report."""
from __future__ import annotations
import argparse
from html import escape
import json
from pathlib import Path
from tools.navigator_a3.eps_identity import combine_inventories, file_sha256


def generate(donor_path, a2_path, donor_manifest, a2_manifest, output):
  donor, a2 = [json.loads(p.read_text()) for p in (donor_path, a2_path)]
  expected_donor = json.loads(donor_manifest.read_text())['files']
  expected_a2 = json.loads(a2_manifest.read_text())
  verified=[]
  for name, data, expected in [('donor',donor,expected_donor),('a2',a2,expected_a2)]:
    expected_by_segment = {(int(Path(r['file']).parent.name.split('--')[-1]) if name=='donor' else r['segment']):r for r in expected}
    actual_by_segment = {int(Path(r['file']).parent.name.split('--')[-1]):r for r in data['inputs']}
    if len(actual_by_segment)!=len(data['inputs']) or set(actual_by_segment)!=set(expected_by_segment):
      raise ValueError(f'{name}: duplicate/missing/unexpected segments')
    for segment,r in actual_by_segment.items():
      if r['sha256'] != expected_by_segment[segment]['sha256']:
        raise ValueError(f'{name}: raw hash mismatch at segment {segment}')
      verified.append({'dataset':name,'segment':segment,'file':r['file'],'sha256':r['sha256']})
  combined=combine_inventories([donor,a2])
  combined['raw_manifest_reconciliation']={'verified_files':verified,'expected_manifests':[
    {'file':str(p),'sha256':file_sha256(p)} for p in (donor_manifest,a2_manifest)]}
  combined['software_number_did']='f188'
  combined['f110_interpretation']='Diagnostic specification reference according to independent tool; manufacturer PSCM semantics unverified.'
  combined['physical_envelope_established']=False
  combined['report_source_sha256']=file_sha256(Path(__file__))
  output.mkdir(parents=True,exist_ok=True)
  (output/'identity-comparison.json').write_text(json.dumps(combined,indent=2)+'\n')
  route_rows=[]
  for route,details in combined['routes'].items():
    inventory=next(d for d in (donor,a2) if route in d['routes'])
    count=sum(r['carParams_publications'] for r in inventory['inputs'])
    route_rows.append(f'<tr><td>{escape(route)}</td><td>{len(inventory["inputs"])}</td><td>{count}</td><td>{len(details["candidates"])}</td><td>{escape(details["status"])}</td></tr>')
  comparison_rows=[]
  for row in combined['comparisons']:
    values=set(v for group in row['values_hex'].values() for v in group)
    display=[]
    for raw in sorted(values):
      payload=bytes.fromhex(raw).rstrip(b'\0')
      display.append(payload.decode('ascii') if all(32<=x<127 for x in payload) else payload.hex())
    meaning={'f188':'ECU software number; not hardware or complete calibration identity','f110':'Separate F110 payload; DS prefix retained; interpretation qualified','de00':'Opaque diagnostic bytes; no physical interpretation','de01':'Opaque diagnostic bytes; no physical interpretation','de02':'Opaque diagnostic bytes; no physical interpretation'}.get(row['did'],'Unclassified diagnostic value')
    comparison_rows.append('<tr>'+''.join(f'<td>{escape(str(x))}</td>' for x in (row['did'].upper(),'; '.join(display),row['status'],meaning))+'</tr>')
  observations=[]
  for route,details in combined['routes'].items():
    observations.append(f'<details><summary>{escape(route)}: complete candidate metadata</summary><pre>{escape(json.dumps(details,indent=2))}</pre></details>')
  matched=sum(r['status']=='same_observed_value' for r in combined['comparisons'])
  html='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Navigator EPS identity and remaining evidence</title><style>body{font:16px/1.55 system-ui;background:#eef2f5;color:#162d3e;margin:0}main{max-width:1120px;margin:auto;padding:32px 22px}h1{font-size:34px;line-height:1.2}section{background:white;border-radius:12px;padding:24px;margin:20px 0}table{border-collapse:collapse;width:100%;font-size:14px}td,th{border-bottom:1px solid #dbe2e8;padding:10px;text-align:left;vertical-align:top}a{color:#1458a0}pre{font-size:12px;white-space:pre-wrap;overflow-wrap:anywhere}summary{cursor:pointer;padding:12px 0;font-weight:600}.scroll{overflow:auto}.tag{font-weight:700;color:#805a09}</style><main><p class="tag">SAVED LOGS + PUBLIC SOURCE EVIDENCE • NO DEVICE CHANGES</p><h1>EPS identity comparison across both routes</h1>
<section><h2>What we established</h2><p>FILES raw log segments reconcile with their previously preserved hashes. MATCHED diagnostic fields have matching observed values across BluePilot and A2/Chestnut. The F188 software-number field and separate F110 payload are decoded below.</p><p>These are carParams firmware-scan metadata inside raw logs. Publications may repeat cached scan results. Their native timestamps date the publication, not a fresh diagnostic query. Matching identifiers do not establish binary, calibration or physical-response equivalence.</p><p><a href="identity-comparison.json">Comparison and raw provenance</a> · <a href="EPS_PUBLIC_EVIDENCE.md">Source-linked research</a> · <a href="tests-final.txt">Tests</a> · <a href="delivery-manifest.json">Source pins and hashes</a> · <a href="run-extraction.py">Reproduction</a></p></section>
<section><h2>What the fields mean</h2><p>Trailing NUL padding is hidden only in this display. Complete original bytes, requests, scanner labels, buses and padding remain in the JSON. Scanner labels such as Mazda or Hyundai identify the query configuration; they are not the vehicle manufacturer.</p><div class="scroll"><table><thead><tr><th>DID</th><th>Displayed value</th><th>Comparison</th><th>Interpretation</th></tr></thead><tbody>COMPARISON_ROWS</tbody></table></div><p>F188 is named ECU software number in the <a href="https://github.com/BluePilotDev/bluepilot/blob/3210caa02d9e09b46d08be490f689edb23b22b20/opendbc_repo/opendbc/car/uds.py">pinned UDS implementation</a>. An <a href="https://github.com/tonesto7/fordpass-scriptable/blob/798b5d13d54c4f7a8f35464ca001dec08c28778b/module_did_desc.json">independent FordPass tool</a> calls F110 a diagnostic-specification part number; that is not official confirmation for this PSCM.</p></section>
<section><h2>What remains unsupported</h2><table><thead><tr><th>Question</th><th>Evidence and limit</th></tr></thead><tbody><tr><td>Same recorded software identity?</td><td>The per-DID comparison above gives the result. The reference NL14-14D003-AE number appears under both Expedition and Ranger fingerprints; it is not a unique rack/calibration identity.</td></tr><tr><td>BluePilot curvature filter at 0x101B0B60?</td><td>Published developer claim. No matching executable/calibration filename or hash supplied in the article; applicability to this Navigator remains unknown.</td></tr><tr><td>Path-angle mapping and limits?</td><td>Donor code describes its command construction. Neither the code nor this identity match establishes a safe physical command-to-response envelope.</td></tr><tr><td>Inactive mode and driver handoff?</td><td>No corresponding firmware transition evidence located. An active zero command, inactive message, driver override and rejected packet must remain distinct.</td></tr></tbody></table><p><a href="https://bluepilot.dev/announcements/?post=bluepilot-7-0-the-return-of-angle-control-it-wasnt-the-models-fault">BluePilot’s published explanation</a> remains useful research context, not a verified Navigator firmware contract.</p></section>
<section><h2>The exact next missing input</h2><p>Obtain an existing module-identification/session export for this PSCM and the authorized application/calibration package manifest matching its installed identity, including filenames and hashes. Separately, BluePilot’s analyzed executable/calibration hashes and relevant disassembly would let us compare its claims directly.</p><p>No active-steering code change is justified by the identifier match alone. Once the corresponding artifacts are available, the next work is to trace path-angle processing and inactive/override/re-entry transitions in those exact binaries before designing independent final-byte limits. Another general road drive would not supply those missing internal definitions.</p><p>The official <a href="https://www.motorcraftservice.com/Diagnostic/HelmSupport?country=USA&amp;language=EN-US&amp;categoryId=286&amp;channelId=46">Motorcraft diagnostic licensing page</a> documents a legitimate calibration-access route; no license was purchased and no matching firmware package was obtained here. Do not flash merely to collect evidence.</p></section>
<section><h2>Coverage and retained provenance</h2><table><thead><tr><th>Route</th><th>Raw files</th><th>carParams publications</th><th>Distinct metadata candidates</th><th>Status</th></tr></thead><tbody>ROUTE_ROWS</tbody></table><p>Publication counts are not independent query counts. All candidate observations retain native logMonoTime, validity, segment hash, message index and entry index.</p>OBSERVATIONS</section></main></html>'''
  html=html.replace('FILES',str(len(verified))).replace('MATCHED',str(matched)).replace('COMPARISON_ROWS',''.join(comparison_rows)).replace('ROUTE_ROWS',''.join(route_rows)).replace('OBSERVATIONS',''.join(observations))
  (output/'index.html').write_text(html)
  return {'verified_files':len(verified),'matched_dids':matched,'routes':{r:x['status'] for r,x in combined['routes'].items()}}

if __name__=='__main__':
  p=argparse.ArgumentParser(description=__doc__)
  for name in ('donor','a2','donor-manifest','a2-manifest','output'):
    p.add_argument('--'+name,type=Path,required=True)
  a=p.parse_args()
  print(json.dumps(generate(a.donor,a.a2,a.donor_manifest,a.a2_manifest,a.output),indent=2))

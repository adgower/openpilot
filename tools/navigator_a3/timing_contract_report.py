"""Render a local report from the compiled synthetic timing comparison."""
from __future__ import annotations
import argparse
from html import escape
import json
from pathlib import Path


def render(source: Path, output: Path):
  data = json.loads(source.read_text())
  compact = {name: {variant: [r for r in rows if r['event']['tx']]
                   for variant, rows in variants.items()}
             for name, variants in data['cases'].items()}
  counts = {v: {'accepted': 0, 'rejected': 0} for v in next(iter(compact.values()))}
  differences = 0
  rows = []
  for name, variants in compact.items():
    historical, revised = variants['strict-historical'], variants['aggregate']
    delta = sum(a['accepted'] != b['accepted'] for a, b in zip(historical, revised, strict=True))
    differences += delta
    row = [escape(name)]
    for label, events in variants.items():
      accepted = sum(e['accepted'] for e in events)
      counts[label]['accepted'] += accepted
      counts[label]['rejected'] += len(events) - accepted
      row.append(f'{accepted} / {len(events)}')
    row.append(str(delta))
    rows.append('<tr>' + ''.join(f'<td>{c}</td>' for c in row) + '</tr>')
  content = '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Navigator • corrected Angle timing contract</title>
<style>body{font:16px/1.55 system-ui;margin:0;background:#eef2f6;color:#182737}main{max-width:1140px;margin:auto;padding:35px 24px}h1{font-size:34px;line-height:1.2}section{background:white;padding:24px;margin:22px 0;border-radius:12px}a{color:#1458a0}table{border-collapse:collapse;width:100%;font-size:13px}td,th{padding:8px;border-bottom:1px solid #dce3e9;text-align:left}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}select{padding:10px;max-width:100%}.good{color:#146848}.bad{color:#a12636}.scroll{overflow:auto}.tag{color:#704809;font-weight:700}svg{width:100%;min-width:640px}summary{cursor:pointer;font-weight:600}</style>
<main><p class="tag">HOST-ONLY EXPERIMENT • ACTUAL A2 STEERING UNCHANGED</p><h1>Angle timing contract: compiled comparison</h1>
<section><h2>What changed</h2><p>The historical evaluator imposed a strict 50 ms minimum interval that was our own experimental choice. The new evaluator reuses the pinned upstream aggregate message-rate check, already present beneath that extra guard, and preserves accepted command references after rejection.</p>
<p>The shadow scheduler remains unchanged. Acceptance below is a compiled software result, not EPS execution, measured steering improvement or approval for active Angle.</p><p><strong>The proposal-derived 49/51 ms fixture: HOST_AGG admitted with aggregate timing, versus HOST_OLD historically.</strong> Ordinary bounded jitter passes; overspeed, malformed packets and independent command violations remain checked.</p><p><strong>DIFFCOUNT admission differences across CASECOUNT synthetic cases.</strong> Counts include whole-sequence consequences; a later difference need not be caused directly by timing.</p>
<p><a href="admission/aggregate-timing-cases.json">Complete machine-readable traces</a> · <a href="TIMING_CONTRACT.md">Constraint origins and missing physical evidence</a> · <a href="tests-final.txt">Full test results</a> · <a href="delivery-manifest.json">Pins and hashes</a> · <a href="run-verification.py">Verification command</a></p></section>
<section><h2>Inspect identical packet sequences</h2><p>Select a fixture. Dots show each transmitted test packet by sequence index; the table preserves its supplied native microsecond timer. Timer-wrap cases deliberately cross the 32-bit boundary. Green means admitted, red means rejected.</p><label for="case">Fixture </label><select id="case"></select><div class="scroll"><svg id="plot" viewBox="0 0 1040 150" role="img" aria-label="Historical and aggregate admissions by packet index"></svg></div><div class="scroll"><table><thead><tr><th>TX index</th><th>Timer µs</th><th>Payload</th><th>Historical</th><th>Aggregate</th><th>Reasons / state</th></tr></thead><tbody id="timeline"></tbody></table></div></section>
<section><h2>Case totals</h2><p>Each cell is accepted / total TX packets. The historical implementation is preserved byte-for-byte. Plain and instrumented builds are compared by the audit.</p><div class="scroll"><table><thead><tr><th>Fixture</th>VARIANT_HEADERS<th>Admission differences</th></tr></thead><tbody>CASE_ROWS</tbody></table></div></section>
<section><h2>What this does not establish</h2><p>Actual recorded A2 rejections retain their original attribution limits. None of these synthetic commands was transmitted to the vehicle. Short bursts allowed by the upstream algorithm are not permission to increase control authority. The independent production gate still rejects nonneutral path angle.</p><p>The next activation evidence must connect identifiable Navigator PSCM firmware/calibration to command-to-response bounds and establish driver handoff, inactive intervals and bounded re-entry. The gain formula, donor acceptance and returned CAN frames cannot establish those facts.</p><p><a href="https://github.com/sunnypilot/opendbc/blob/f95f996f5917dcbbf2e32fe51b606a24cf836af6/opendbc/safety/lateral.h#L177-L196">Pinned aggregate algorithm</a> · <a href="../scheduler-fix/index.html">Preserved historical scheduler report</a></p></section></main>
<script id="data" type="application/json">DATA_JSON</script><script>
const data=JSON.parse(document.getElementById('data').textContent),select=document.getElementById('case');
for(const name of Object.keys(data)){const o=document.createElement('option');o.value=name;o.textContent=name;select.append(o)}
function show(){const c=data[select.value],old=c['strict-historical'],now=c.aggregate,svg=document.getElementById('plot');svg.replaceChildren();const NS='http://www.w3.org/2000/svg';
for(const [k,label] of ['strict-historical','aggregate'].entries()){const t=document.createElementNS(NS,'text');t.setAttribute('x',0);t.setAttribute('y',35+k*70);t.textContent=label;svg.append(t);c[label].forEach((r,i)=>{const dot=document.createElementNS(NS,'circle');dot.setAttribute('cx',175+i*840/Math.max(1,now.length-1));dot.setAttribute('cy',30+k*70);dot.setAttribute('r',5);dot.setAttribute('fill',r.accepted?'#146848':'#a12636');const title=document.createElementNS(NS,'title');title.textContent=`TX ${i}: ${r.event.time_us} µs`;dot.append(title);svg.append(dot)})}
const body=document.getElementById('timeline');body.replaceChildren();now.forEach((r,i)=>{const tr=document.createElement('tr');for(const v of [i,r.event.time_us,r.event.data_hex,old[i].accepted?'admit':'reject',r.accepted?'admit':'reject']){const td=document.createElement('td');td.textContent=v;tr.append(td)}const td=document.createElement('td'),d=document.createElement('details'),s=document.createElement('summary'),p=document.createElement('pre');s.textContent=`Historical: ${old[i].reason_names.join(', ')||(old[i].accepted?'admitted':'unattributed rejection')}; Aggregate: ${r.reason_names.join(', ')||(r.accepted?'admitted':'unattributed rejection')}`;p.textContent=JSON.stringify({historical:{reasons:old[i].reason_names,failed_checks:old[i].failed_checks,before:old[i].before,after:old[i].after},aggregate:{failed_checks:r.failed_checks,before:r.before,after:r.after}},null,2);d.append(s,p);td.append(d);tr.append(td);body.append(tr)})}
select.addEventListener('change',show);show();</script></html>'''
  host = compact['host-jitter-49-51ms']
  for marker, variant in [('HOST_AGG', 'aggregate'), ('HOST_OLD', 'strict-historical')]:
    events = host[variant]
    content = content.replace(marker, f"{sum(e['accepted'] for e in events)}/{len(events)}")
  content = content.replace('DIFFCOUNT', str(differences)).replace('CASECOUNT', str(len(compact)))
  content = content.replace('VARIANT_HEADERS', ''.join(f'<th>{escape(v)}</th>' for v in counts))
  content = content.replace('CASE_ROWS', ''.join(rows)).replace('DATA_JSON', json.dumps(compact).replace('<', '\\u003c'))
  output.write_text(content)
  return {'cases': len(compact), 'admission_differences': differences, 'counts': counts}


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--input', type=Path, required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  print(json.dumps(render(args.input, args.output), indent=2))

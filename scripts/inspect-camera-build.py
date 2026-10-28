# Inspect staged RTL, resource use and timing; eligibility does not mean every timing check passed.
import argparse, hashlib, html, json, re
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--build',required=True);p.add_argument('--output',required=True);a=p.parse_args()
root=Path(__file__).resolve().parent.parent;build=Path(a.build);out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
pnr=build/'impl/pnr';timing=(pnr/'OV5640_HDMI720P_tr_content.html').read_text(encoding='utf-8')
def rows(name):
 if '<h3><a name="'+name+'">' not in timing:return []
 table=timing.split('<h3><a name="'+name+'">')[1].split('</table>')[0]; result=[]
 for row in re.findall(r'<tr[^>]*>(.*?)</tr>',table,re.S):
  vals=re.findall(r'<td[^>]*>(.*?)</td>',row,re.S)
  if vals:result.append([html.unescape(re.sub('<[^>]+>','',v)).strip() for v in vals])
 return result
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
counts={k:int(re.search(re.escape(k)+r'</td>\s*<td>(.*?)</td>',timing,re.S)[1]) for k in ['Numbers of Setup Violated Endpoints','Numbers of Hold Violated Endpoints']}
resource=(pnr/'OV5640_HDMI720P.rpt.txt').read_text(encoding='utf-8'); usage={k:re.search(r'^\s*'+k+r'\s*\|\s*([0-9.]+/[0-9.]+)',resource,re.M)[1] for k in ['Logic','Register','CLS','BSRAM','DSP']}
variant=json.loads((build/'build_variant.json').read_text(encoding='utf-8-sig')); comparisons=[]
for src in (root/'fpga/src').rglob('*'):
 if not src.is_file() or src.suffix.lower() not in ('.v','.cst','.sdc'):continue
 rel=src.relative_to(root/'fpga');staged=build/rel;expected=src.read_bytes()
 if rel.as_posix()=='src/top.v' and variant.get('camera_uart'):
  expected=expected.replace(b'parameter integer SERVO_UART_ENABLE = 0',b'parameter integer SERVO_UART_ENABLE = 1')
  if variant.get('shape_diagnostic'):
   expected=expected.replace(b'.SHOW_SERVO_DEBUG(SERVO_UART_ENABLE ? 0 : SERVO_DIAGNOSTICS)',b'.SHOW_SERVO_DEBUG(2)')
 comparisons.append(dict(source=str(src.relative_to(root)),source_sha256=digest(src),staged_sha256=digest(staged),expected_staged_matches=staged.read_bytes().replace(b'\r\n',b'\n')==expected.replace(b'\r\n',b'\n'),line_endings_normalized=True))
for staged in (build/'src/shared_algorithm').glob('*.v'):
 src=root/'algorithm_sim/rtl'/staged.name
 if not src.exists():src=root/'servo_uart/src'/staged.name
 comparisons.append(dict(source=str(src.relative_to(root)),source_sha256=digest(src),staged_sha256=digest(staged),expected_staged_matches=digest(src)==digest(staged)))
setup=rows('Setup_Slack_Table');removal=rows('Removal_Slack_Table')
result=dict(build_directory=str(build),variant=variant,counts=counts,resources=usage,minimum_setup_slack_ns=min(float(r[1]) for r in setup),minimum_removal_slack_ns=min((float(r[1]) for r in removal),default=None),worst_setup_paths=setup[:8],removal_paths=removal,source_comparisons=comparisons,source_matches=all(c['expected_staged_matches'] for c in comparisons),bitstream_sha256=digest(pnr/'OV5640_HDMI720P.fs'),flash_written=False)
result['all_timing_passed']=counts['Numbers of Setup Violated Endpoints']==0 and counts['Numbers of Hold Violated Endpoints']==0
removal_bad=[r for r in removal if float(r[1])<0]
result['known_reset_removal_only']=bool(removal_bad) and all('cmos_reset_gen_m0/cnt_' in r[3] and 'CLEAR' in r[3] for r in removal_bad) and counts['Numbers of Hold Violated Endpoints']==len({r[3] for r in removal_bad}) and all(float(r[1])>=0 for r in rows('Hold_Slack_Table'))
result['eligible_for_sram_test']=(result['all_timing_passed'] or result['known_reset_removal_only']) and result['source_matches'] and counts['Numbers of Setup Violated Endpoints']==0 and variant.get('camera_uart') and not variant.get('servo_enabled_at_boot')
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps({k:v for k,v in result.items() if k not in ['source_comparisons','worst_setup_paths','removal_paths','variant']},indent=2))
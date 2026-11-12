"""Reproduce drive-release pitch kick and check the simulator's soft default."""
import argparse
import json
import pathlib
import subprocess

p=argparse.ArgumentParser()
p.add_argument('--runner',required=True)
p.add_argument('--output',default='/tmp/rb-drive-stop-regression')
a=p.parse_args();out=pathlib.Path(a.output);out.mkdir(parents=True,exist_ok=True)

def run(name,speed=.3,height=.14,unit=648,override=None):
    command=[a.runner,'--model','model/dynamic.xml','--rate','3000','--start-key','home',
        '--imu-model','1','--imu-unit',str(unit),'--run-for','38','--at','0.3:STANDUP',
        '--at',f'16:HEIGHT {height}','--at',f'25:DRIVE {speed} 0','--at','29:STOP']
    if override:command+=['--control',override]
    result=subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True)
    (out/(name+'.log')).write_text(result.stdout)
    states=[];views=[]
    for line in result.stdout.splitlines():
        if line.startswith(('STATE|','VIEW|')):
            fields=dict(s.split('=',1) for s in line.split('|')[1:] if '=' in s)
            (states if line.startswith('STATE|') else views).append(fields)
    after=[s for s in states if 29<=float(s['t'])<34]
    sign=1 if speed>0 else -1
    positions=[sign*float(v['x']) for v in views if 29<=float(v['t'])<34]
    peak_rate=max(abs(float(s['pitch_rate'])) for s in after)
    stats={'peak_pitch_rate_rad_s':peak_rate,'reverse_from_furthest_m':max(positions)-positions[-1],
           'mode':states[-1]['mode'],'fault':states[-1]['fault_reason']}
    assert states[-1]['mode']=='Balancing' and all(s['mode']!='Fault' for s in after),stats
    assert peak_rate<1.0,stats
    return stats

results={}
results['old_profile']=run('old_profile',override='plan_accel=0.9,plan_jerk=10')
results['default']=run('default')
assert results['default']['peak_pitch_rate_rad_s']<.7*results['old_profile']['peak_pitch_rate_rad_s'],results
results['reverse']=run('reverse',speed=-.3)
results['faster']=run('faster',speed=.6)
results['tall']=run('tall',height=.20)
results['other_imu']=run('other_imu',unit=208)
(out/'summary.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))

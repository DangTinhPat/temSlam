"""Lifecycle owned by the existing simulator backend."""
import json,os,signal,subprocess,time
from pathlib import Path


def deltar_python():
    return os.environ.get('DELTAR_PYTHON', str(Path.home()/'.venvs/deltar/bin/python'))

class LiveSession:
    def __init__(self,root,session):
        self.session=session;self.children=[];self.logs=[]
        script=root/'simulation/demo/live/workers.py'
        env=os.environ.copy();env.pop('PYTHONPATH',None);env['MUJOCO_GL']='egl'
        try:
            headless = os.environ.get('RB1_HEADLESS') == '1'
            for stage in (['infer','slam'] if headless else ['infer','slam','mesh']):
                command=([str(root/'vslam/run_python.sh')] if stage=='slam'
                         else [deltar_python()])+[str(script),stage,str(session)]
                log=open(session/(stage+'.log'),'w');self.logs.append(log)
                self.children.append((stage,subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)))
            if not headless:
                log=open(session/'rviz.log','w');self.logs.append(log)
                self.children.append(('rviz',subprocess.Popen([str(root/'simulation/demo/live/rviz.sh'),str(session)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)))
        except Exception:
            self.close();raise

    def status(self):
        result=[]
        for stage,child in self.children:
            state={}
            try: state=json.loads((self.session/(stage+'_status.json')).read_text());text=state['message']
            except FileNotFoundError: text='Đang khởi động'
            if stage=='infer' and 'inference_ms' in state:
                text+=f" ({state['inference_ms']:.0f} ms)"
            if child.poll() is not None: text=f'Đã dừng ({child.returncode}): '+text
            if stage!='rviz': result.append(stage+': '+text)
        return ' | '.join(result)

    def ready(self):
        state = self.tracking_status()
        return (state.get('keyframes', 0) >= 3 and time.time()-state.get('wall_time', 0) < 2
                and state.get('tracking_usable', True)
                and not state.get('message', '').startswith('LỖI'))

    def tracking_status(self):
        try:
            return json.loads((self.session/'slam_status.json').read_text())
        except FileNotFoundError:
            return {}

    def scan_feedback(self):
        state = self.tracking_status()
        if (time.time()-state.get('wall_time', 0) < 5 and 'head_yaw_rad' in state
                and state.get('tracking_usable', True)):
            return state
        return None

    def failure_reason(self):
        for stage, child in self.children:
            if stage not in ('infer', 'slam'):
                continue
            try:
                state = json.loads((self.session/(stage+'_status.json')).read_text())
                if state['message'].startswith('LỖI'):
                    return stage + ': ' + state['message']
            except (FileNotFoundError, KeyError):
                pass
            if child.poll() is not None:
                return f'{stage} đã dừng ({child.returncode})'
        return None

    def close(self):
        (self.session/'STOP').touch()
        # Tracking saves its database on a clean stop; terminate only owned process groups afterwards.
        for name,child in self.children:
            if name=='slam':
                # shortcut: allow 60s for up to 4096 views; report timeout if larger sessions need more.
                try:
                    clean=child.wait(timeout=60)==0
                    reason='completed' if clean else 'SLAM worker failed'
                except subprocess.TimeoutExpired:
                    clean=False;reason='Timed out waiting for final SLAM history/map export'
                (self.session/'slam_shutdown.json').write_text(json.dumps(dict(clean_stop=clean,reason=reason))+'\n')
        for _,child in self.children:
            if child.poll() is None:
                try: os.killpg(child.pid,signal.SIGTERM)
                except ProcessLookupError: pass
        for _,child in self.children:
            try: child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in self.logs: log.close()

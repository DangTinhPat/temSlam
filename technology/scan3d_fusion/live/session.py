"""Lifecycle owned by the existing simulator backend."""
import json,os,signal,subprocess,time

class LiveSession:
    def __init__(self,root,session):
        self.session=session;self.children=[];self.logs=[]
        script=root/'technology/scan3d_fusion/live/workers.py'
        env=os.environ.copy();env.pop('PYTHONPATH',None);env['MUJOCO_GL']='egl'
        try:
            headless = os.environ.get('RB1_HEADLESS') == '1'
            for stage in (['infer','slam'] if headless else ['infer','slam','mesh']):
                command=([str(root/'technology/scan3d_fusion/mapping/cuvslam/run_python.sh')] if stage=='slam'
                         else ['/home/dvt/.venvs/deltar/bin/python'])+[str(script),stage,str(session)]
                log=open(session/(stage+'.log'),'w');self.logs.append(log)
                self.children.append((stage,subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)))
            if not headless:
                log=open(session/'rviz.log','w');self.logs.append(log)
                self.children.append(('rviz',subprocess.Popen([str(root/'technology/scan3d_fusion/live/rviz.sh'),str(session)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)))
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
        try:
            state=json.loads((self.session/'slam_status.json').read_text())
            return (state.get('keyframes',0)>=3 and time.time()-state['wall_time']<2
                    and not state['message'].startswith('LỖI'))
        except (FileNotFoundError,KeyError): return False

    def scan_feedback(self):
        try:
            state=json.loads((self.session/'slam_status.json').read_text())
            if time.time()-state['wall_time']<5 and 'head_yaw_rad' in state:
                return state
        except (FileNotFoundError,KeyError):
            pass
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
                try: child.wait(timeout=3)
                except subprocess.TimeoutExpired: pass
        for _,child in self.children:
            if child.poll() is None:
                try: os.killpg(child.pid,signal.SIGTERM)
                except ProcessLookupError: pass
        for _,child in self.children:
            try: child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in self.logs: log.close()

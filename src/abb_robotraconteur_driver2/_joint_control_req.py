from . import _rws
import threading
from contextlib import suppress
import traceback
class JointControlReq:
    def __init__(self, rws: _rws.ABBRobotRWSImpl):
        self.rws = rws
        self.error = False
        self.error_obj = None
        self.started = False
        self.mp_req = None
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._stopped = threading.Event()

    @property
    def ready(self):
        if self.error:
            return False
        req = self.mp_req
        if req is None:
            return False
        return self.rws.is_motion_program_running(req) and self.rws.motion_program_state == _rws.MotionProgramState.running_egm_joint_control
    
    def start(self):
        with self._lock:
            self._thread = threading.Thread(target=self._run)
            self._thread.daemon = True
            self._thread.start()            

    def stop(self):
        self._stop.set()

    def stop_join(self, timeout = 0.1):
        with self._lock:
            self._stop.set()
            t = self._thread
            req = self.mp_req
        if t is not None:
            t.join(timeout)
            self._stopped.wait(timeout=1)
        if req is not None:
            assert not self.rws.is_motion_program_running(req)

    def _run(self):
        try:
            self.mp_req = self.rws.start_joint_control(self._mp_handler, False)
        except Exception as e:
            self.error = True
            self.error_obj = e
            self._stopped.set()
            return

        self._stop.wait()

        if not self.error:
            req = self.mp_req
            if req is not None:
                # with suppress(Exception):
                try:
                    self.rws.stop_motion_program(self.mp_req)
                except:
                    traceback.print_exc()


    def _mp_handler(self, mp, mp_state, mp_param):
        if mp_state in (_rws.MotionProgramState.error, _rws.MotionProgramState.idle, _rws.MotionProgramState.complete):
            self.error = True
            self.error_obj = Exception("Joint control stopped")
            self._stop.set()
            self._stopped.set()

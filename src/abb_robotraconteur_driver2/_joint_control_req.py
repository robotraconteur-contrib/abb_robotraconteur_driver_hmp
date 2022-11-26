from . import _rws
import threading
from contextlib import suppress

class JointControlReq:
    def __init__(self, rws: _rws.ABBRobotRWSImpl):
        self.rws = rws
        self.error = False
        self.error_obj = None
        self.started = False
        self.lock = threading.Lock()
        self.mp_req = None

    @property
    def ready(self):
        if self.error:
            return False
        req = self.mp_req
        if req is None:
            return False
        return self.rws.is_motion_program_running(req) and self.rws.motion_program_state == _rws.MotionProgramState.running_egm_joint_control
    
    def start(self):
        with self.lock:
            try:
                self.mp_req = self.rws.start_joint_control(self._mp_handler, False)
            except Exception as e:
                self.error = True
                self.error_obj = e

    def stop(self):
        with self.lock:
            req = self.mp_req
            if req is not None:
                with suppress(Exception):
                    self.rws.stop_motion_program(self.mp_req)

    def _mp_handler(self, mp, mp_state, mp_param):
        if mp_state in (_rws.MotionProgramState.error, _rws.MotionProgramState.idle, _rws.MotionProgramState.complete):
            self.error = True
            self.error_obj = Exception("Joint control stopped")

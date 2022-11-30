from . import _rws
import threading
from contextlib import suppress
import traceback
import asyncio
import concurrent.futures
class JointControlReq:
    def __init__(self, rws: _rws.ABBRobotRWSImpl):
        self.rws = rws
        self.error = False
        self.error_obj = None
        self._mp_future = None
        self._mp_task = None
        self._lock = threading.Lock()
        self._stopped = False
        self._mp_state = _rws.MotionProgramState.idle

    @property
    def ready(self):
        if self.error:
            return False
        fut = self._mp_future
        if fut is None or fut.done():
            return False
        return self._mp_state == _rws.MotionProgramState.running_egm_joint_control
    
    def start(self):
        with self._lock:
            self._mp_future = asyncio.run_coroutine_threadsafe(self._run_task(), self.rws.loop)

    def stop(self):
        with self._lock:
            self._stopped = True
            if self._mp_task:
                self.rws.loop.call_soon_threadsafe(lambda: self._mp_task.cancel())

    def stop_join(self, timeout = 0.1):
        with self._lock:
            self._stopped = True
            t = self._mp_task
            if t is not None:
                self.rws.loop.call_soon_threadsafe(lambda: self._mp_task.cancel())
            fut = self._mp_future
            self._mp_future = None
        if fut is not None:
            with suppress(concurrent.futures.CancelledError):
                fut.result(timeout = timeout)                    
            assert fut.done()

    async def _run_task(self):
        try:            
            mp_task, mp_status = self.rws.start_joint_control(start_timeout = 0.05, enable_motion_logging=False)
            with self._lock:
                self._mp_task = mp_task
                if self._stopped:
                    self._mp_task.cancel()
            while True:
                state, data = await mp_status.get()
                self._mp_state = state
                if state in (_rws.MotionProgramState.complete, _rws.MotionProgramState.error,
                     _rws.MotionProgramState.cancelled) \
                    or mp_task.done():
                    break
            await mp_task

        except Exception as e:
            self._mp_state == _rws.MotionProgramState.error
            self.error_obj = e
            traceback.print_exc()
        finally:
            self.error = True
            if self._mp_state != _rws.MotionProgramState.error:
                self._mp_state = _rws.MotionProgramState.complete

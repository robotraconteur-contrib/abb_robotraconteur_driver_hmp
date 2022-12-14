from . import _rws
import threading
from contextlib import suppress
import traceback
import asyncio
import concurrent.futures
import queue
class JointControlReq:
    def __init__(self, abb_robot_impl, rws: _rws.ABBRobotRWSImpl):
        self.abb_robot_impl = abb_robot_impl
        self.rws = rws
        self.error_obj = None
        self._lock = threading.Lock()
        self._closed = False
        self._mp_state = _rws.MotionProgramState.idle
        self._mp_task = None
        self._enabled = False
        self._cv = asyncio.Condition()
        self._enable_queue = queue.Queue()
        self._enable_seqno_ctr = 1
        self._last_enable_seqno = -1
        self._enable_th_cv = threading.Condition()
        self._loop_set_enabled_last = False

    def loop_set_enabled(self, en):
        if self._loop_set_enabled_last != en:
            self._loop_set_enabled_last = en
            if en:
                self.enable()
            else:
                self.disable()

    @property
    def ready(self):
        if self.error_obj is not None or self._closed:
            return False
        return self._mp_state == _rws.MotionProgramState.running_egm_joint_control
    
    def start(self):
        with self._lock:
            assert not self._closed, "JointControlReq closed"
            self._mp_future = asyncio.run_coroutine_threadsafe(self._run_task(), self.rws.loop)

    def close(self):
        with self._lock:
            self._closed = True
            if self._mp_task:
                self.rws.loop.call_soon_threadsafe(lambda: self._mp_task.cancel())

    async def _do_enable_aio(self):
        seqno = None
        async with self._cv:
            with suppress(queue.Empty):
                while True:
                    seqno, en = self._enable_queue.get_nowait()
            if seqno is not None:
                self._enabled = en
                self._cv.notify_all()
        
        if seqno is not None:
            with self._enable_th_cv:
                self._last_enable_seqno = seqno
                self._enable_th_cv.notify_all()

    def _do_enable(self, en):
        self._enable_seqno_ctr += 1
        seqno = self._enable_seqno_ctr
        self._enable_queue.put((seqno, en))
        asyncio.run_coroutine_threadsafe(self._do_enable_aio(), self.rws.loop)
        return seqno
    
    def enable(self, timeout = 0.0):
        with self._lock:            
            assert not self._closed
            seqno = self._do_enable(True)
        if timeout > 0:
            with self._enable_th_cv:
                self._enable_th_cv.wait_for(lambda: self._last_enable_seqno >= seqno and \
                    (self._mp_state == _rws.MotionProgramState.running_egm_joint_control or \
                    self._last_enable_seqno > seqno), timeout = timeout)

    def disable(self, timeout = 0.0):
        with self._lock:
            seqno = self._do_enable(False)
        if timeout > 0:
            with self._enable_th_cv:
                self._enable_th_cv.wait_for(lambda: self._last_enable_seqno >= seqno and \
                    (self._mp_state == _rws.MotionProgramState.idle or self._last_enable_seqno > seqno), \
                    timeout = timeout)

    async def _run_task(self):
        while not self._closed:
            mp_task = None
            async with self._cv:
                await self._cv.wait_for(lambda: self._enabled or self._closed)
            try:
                self.error_obj = None
                tool = self.abb_robot_impl._current_tool[0]
                payload = self.abb_robot_impl._current_payload[0]
                payload_pose = self.abb_robot_impl._current_payload_pose[0]
                self._request_reload=False
                mp_task, mp_status = self.rws.start_joint_control(start_timeout = 0.05, enable_motion_logging=False, 
                    tool = tool, payload=payload, payload_pose=payload_pose)
                with self._lock:
                    self._mp_task = mp_task
                    if self._closed:
                        self._mp_task.cancel()
                async def state_update():
                    while not self._closed:
                        state, data = await mp_status.get()
                        async with self._cv:
                            self._mp_state = state
                            self._cv.notify_all()
                        if state in (_rws.MotionProgramState.complete, _rws.MotionProgramState.error,
                            _rws.MotionProgramState.cancelled) \
                            or mp_task.done():
                            break
                state_update_task = self.rws.loop.create_task(state_update())
                async with self._cv:
                    await self._cv.wait_for(lambda: not self._enabled or self._closed or self._request_reload or
                    self._mp_state in (_rws.MotionProgramState.complete, _rws.MotionProgramState.error,
                            _rws.MotionProgramState.cancelled
                    ))

                mp_task.cancel()
                await mp_task        
                
            except Exception as e:
                self._mp_state == _rws.MotionProgramState.error
                self.error_obj = e
                traceback.print_exc()
            finally:
                if self._mp_state != _rws.MotionProgramState.error:
                    self._mp_state = _rws.MotionProgramState.complete
                with suppress(Exception, asyncio.TimeoutError):
                    mp_task.cancel()
            try:
                if mp_task is not None:
                    await mp_task
            except (Exception, asyncio.TimeoutError):
                traceback.print_exc()
            self._mp_state = _rws.MotionProgramState.idle
            
            async with self._cv:
                with suppress(asyncio.TimeoutError):
                    if not self._request_reload:
                        await asyncio.wait_for(self._cv.wait_for(lambda: self._closed), timeout = 0.5)

    def request_reload(self):
        async def _do_req():
            async with self._cv:
                self._request_reload = True
                self._cv.notify_all()

        asyncio.run_coroutine_threadsafe(_do_req(), self.rws.loop)

import RobotRaconteur as RR
import numpy as np
import abb_motion_program_exec as abb_exec
from RobotRaconteur.RobotRaconteurPythonUtil import NamedArrayToArray
import threading
import traceback
import time
from . import _rws as rws
import asyncio
from contextlib import suppress
from ._motion_program_conv import rr_motion_program_to_abb

class MotionExecImpl:
    def __init__(self, abb_robot_impl):
        self.abb_robot_impl = abb_robot_impl
        self.rws = abb_robot_impl._rws
        self.node = abb_robot_impl._node

    def execute_motion_program(self, program):

        abb_program = rr_motion_program_to_abb(program, self.abb_robot_impl._rox_robots[0])

        gen = ExecuteMotionProgramGen(self, self.rws, abb_program)

        return gen

    def run_timestep(self, now):
        pass


class ExecuteMotionProgramGen:

    def __init__(self, parent, rws, motion_program, save_log = False):
        self._node = parent.node
        self._parent = parent
        self._rws = rws
        self._motion_program = motion_program
        self._action_status_code = self._node.GetConstants("com.robotraconteur.action")["ActionStatusCode"]
        self._status = self._action_status_code["queued"]
        self._mp_status = self._node.GetStructureType("experimental.robotics.motion_program.MotionProgramStatus")
        self._log_handle = 0
        self._save_log = save_log
        self._lock = threading.Lock()
        
        self._closed = False

        self._mp_lock = asyncio.Lock()
        self._mp_task = None
        self._mp_states = None

        

    
    def AsyncNext(self, handler):
        # with self._lock:
        #     if self._closed:
        #         raise RR.StopIterationException("")
            
        if self._status == self._action_status_code["queued"]:
            async def _start_mp():
                try:
                    mp_task, mp_states = self._rws.execute_motion_program(self._motion_program, 
                        enable_motion_logging = self._save_log)
                    
                    with self._lock:
                        self._mp_task = mp_task
                        self._mp_states = mp_states
                        if self._closed:
                            self._mp_task.cancel()
                    self._status = self._action_status_code["running"]
                except BaseException as e:
                    traceback.print_exc()
                    err = e
                    self._closed = True
                    self._status = self._action_status_code["error"]
                    with suppress(Exception):
                        self._mp_task.cancel()
                    handler(None, err)
                else:
                    ret = self._mp_status()
                    ret.current_command = -1
                    ret.queued_command = -1
                    ret.action_status = self._status
                    handler(ret, None)
            asyncio.run_coroutine_threadsafe(_start_mp(), self._rws.loop)
            return

        if self._status == self._action_status_code["running"]:
            async def _run_mp():
                ret = self._mp_status()
                ret.current_command = -1
                ret.queued_command = -1
                try:
                    state, data = await self._mp_states.get()
                    if state in (rws.MotionProgramState.running, rws.MotionProgramState.running_egm_joint_control,
                        rws.MotionProgramState.running_egm_path_corr, rws.MotionProgramState.running_egm_pose_control):
                        ret.current_command, ret.queued_command = data
                    elif state in (rws.MotionProgramState.starting, rws.MotionProgramState.stopping, 
                        rws.MotionProgramState.stop_requested, rws.MotionProgramState.stopping_egm):
                        pass
                    elif state == rws.MotionProgramState.complete:
                        self._closed = True
                        self._status = self._action_status_code["complete"]
                        # TODO: store recording result
                        raise RR.StopIterationException("")
                    elif state == rws.MotionProgramState.cancelled:
                        raise RR.OperationAbortedException("Motion program cancelled")
                    elif state == rws.MotionProgramState.error:
                        raise data
                    else:
                        raise Exception(f"Unknown motion program state {state}")
                except BaseException as e:
                    traceback.print_exc()
                    self._status = self._action_status_code["error"]
                    err = e
                    self._closed = True
                    handler(None, err)
                else:
                    ret.action_status = self._status
                    handler(ret, None)
            asyncio.run_coroutine_threadsafe(_run_mp(), self._rws.loop)
            return
        raise RR.StopIterationException("")

                
    def Close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            with suppress(Exception):
                self._rws.loop.call_soon_threadsafe(lambda: self._mp_task.cancel())

    def Abort(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            with suppress(Exception):
                self._rws.loop.call_soon_threadsafe(lambda: self._mp_task.cancel())


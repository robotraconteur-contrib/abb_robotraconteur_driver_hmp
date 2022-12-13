
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
import random

class MotionExecImpl:
    def __init__(self, abb_robot_impl):
        self.abb_robot_impl = abb_robot_impl
        self.rws = abb_robot_impl._rws
        self.node = abb_robot_impl._node

    def execute_motion_program(self, program, record = False):

        abb_program = rr_motion_program_to_abb(program, self.abb_robot_impl._rox_robots[0])

        gen = ExecuteMotionProgramGen(self, self.rws, abb_program, record)

        return gen

    def preempt_motion_program(self, program, preempt_number, preempt_cmdnum):
        abb_program = rr_motion_program_to_abb(program, self.abb_robot_impl._rox_robots[0])

        async def _do_preempt():
            await self.rws.preempt_motion_program(abb_program, preempt_number, preempt_cmdnum)
        
        fut = asyncio.run_coroutine_threadsafe(_do_preempt(), self.rws.loop)
        fut.result()


    def run_timestep(self, now):
        pass

    def store_recording(self, rec_handle, rec):
        self.abb_robot_impl._motion_program_recordings[rec_handle] = rec


class ExecuteMotionProgramGen:

    def __init__(self, parent, rws, motion_program, record = False):
        self._node = parent.node
        self._parent = parent
        self._rws = rws
        self._motion_program = motion_program
        self._action_status_code = self._node.GetConstants("com.robotraconteur.action")["ActionStatusCode"]
        self._status = self._action_status_code["queued"]
        self._mp_status = self._node.GetStructureType("experimental.robotics.motion_program.MotionProgramStatus")
        self._log_handle = 0
        self._record = record
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
                        enable_motion_logging = self._record)
                    
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
                ret.current_preempt = -1
                try:
                    state, data = await self._mp_states.get()
                    if state in (rws.MotionProgramState.running, rws.MotionProgramState.running_egm_joint_control,
                        rws.MotionProgramState.running_egm_path_corr, rws.MotionProgramState.running_egm_pose_control):
                        ret.current_command, ret.queued_command, ret.current_preempt = data
                    elif state in (rws.MotionProgramState.starting, rws.MotionProgramState.stopping, 
                        rws.MotionProgramState.stop_requested, rws.MotionProgramState.stopping_egm):
                        pass
                    elif state == rws.MotionProgramState.complete:
                        self._closed = True
                        self._status = self._action_status_code["complete"]
                        # TODO: store recording result
                        if self._record:
                            rec_handle = random.randint(0,0xFFFFFFF)
                            self._parent.store_recording(rec_handle, data)
                            ret.recording_handle = rec_handle
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


class RobotRecordingGen:
    def __init__(self, parent, robot_rec_np, ):
        self.robot_rec_np = robot_rec_np
        self.closed = False
        self.aborted = False
        self.lock = threading.Lock()
        self._node = parent._node
        self._mp_log_part = self._node.GetStructureType("experimental.robotics.motion_program.MotionProgramRecordingPart")

    def Next(self):
        with self.lock:
            if self.aborted:
                raise RR.OperationAbortedException("Log aborted")

            if self.closed:
                raise RR.StopIterationException()

            ret = self._mp_log_part()

            # All current paths expect to be within 10 MB limit
            ret.time = self.robot_rec_np.data[:,0].flatten().astype(np.float64)
            ret.command_number = self.robot_rec_np.data[:,1].flatten().astype(np.int32)
            ret.joints = self.robot_rec_np.data[:,2:].astype(np.float64)
            ret.column_headers = self.robot_rec_np.column_headers

            self.closed = True
            return ret

    def Abort(self):
        self.aborted = True

    def Close(self):
        self.closed = True
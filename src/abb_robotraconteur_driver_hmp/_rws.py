import abb_robot_client.rws as rws
import abb_robot_client.rws_aio as rws_aio
from typing import NamedTuple, Callable
import threading
from enum import IntEnum
import traceback
import abb_motion_program_exec as abb_exec
from abb_motion_program_exec import abb_motion_program_exec_client_aio as abb_exec_aio
import time
import RobotRaconteur as RR
from contextlib import suppress
import asyncio
import json
import re
from . import _motion_program_conv

class ConnectionStatus(IntEnum):
    idle = 0
    connecting = 1
    connected = 2
    disconnected = 3
    error = 4

class MotionProgramState(IntEnum):
    idle = 0
    starting = 1
    running = 2
    running_egm_joint_control = 3
    running_egm_pose_control = 4
    running_egm_path_corr = 5
    stop_requested = 6
    stopping_egm = 7
    stopping = 8
    complete = 9
    error = 10
    cancelled = 11

class ABBRobotRWSImpl:
    def __init__(self, robot_url, robot_username = None, robot_password = None):
        self._robot_url = robot_url
        self._robot_username = robot_username
        self._robot_password = robot_password
        self.connection_status = ConnectionStatus.idle
        self._last_error = None
        self._robot_client = None
        self._user_robot_client = rws.RWS(robot_url, robot_username, robot_password)
        self._robot_mp_client = None

        self.controller_opmode: int = 0
        self.controller_state: int = 0
        self.exec_state: bool = False
        self.exec_error: bool = False
        self.motion_program_state = MotionProgramState.idle

        self._mp_seqno_i = 1

        self._last_contact = time.perf_counter()
        self.exec_motion_program_seqno = -1
        self.exec_motion_program_seqno_started = -1
        self.exec_motion_program_seqno_complete = -1
        self.exec_current_cmd_num = -1
        self.exec_queued_cmd_num = -1
        self.exec_current_preempt_num = -1
        self.exec_egm_active = -1

        self.egm_communication_failure = False
        self._last_egm_reset = 0

        self._stop_all = False

        self._connect_task_fut = None

        self._state_cv = asyncio.Condition()
        self._motion_program_lock = asyncio.Lock()
        self.loop = None

        self._last_clean = 0

        self._file_clean_re=re.compile(r"^.*\-\-\-seqno\-(\d+)\.bin$")


    def start(self):
        self.loop = asyncio.get_event_loop()
        self._connect_task_fut = asyncio.ensure_future(self._connect_task())
        pass

    def close(self):
        if self._connect_task_fut:
            self._connect_task_fut.cancel()

    async def _connect_task(self):
        locked = False
        self.connection_status = ConnectionStatus.idle        
        while True:
            subscription_task = None
            try:
                if self._robot_client is None:
                    self._robot_client = rws_aio.RWS_AIO(self._robot_url, self._robot_username, self._robot_password)
                    self._robot_mp_client = abb_exec_aio.MotionProgramExecClientAIO(abb_client_aio = self._robot_client)

                self.connection_status = ConnectionStatus.connecting
                await self._update_state()
                self._ramdisk = await self._robot_client.get_ramdisk_path()

                await self._robot_client.set_analog_io("motion_program_driver_abort", 1)
                await asyncio.sleep(0.2)
                await self._robot_client.set_analog_io("motion_program_driver_abort", 0)
                
                subscription_task = asyncio.create_task(self._subscription_task(self._robot_client))
                
                await self._update_state()
                self.connection_status = ConnectionStatus.connected

                while True:
                    if time.perf_counter() - self._last_contact > 1:
                        await self._update_state()
                    await asyncio.sleep(1)
                    await self._clean_files()
            except Exception as e:
                # self._handle_error(e)
                traceback.print_exc()                
                self.connection_status = ConnectionStatus.error
            finally:
                if locked:
                    self._motion_program_lock.release()
                if self.connection_status != ConnectionStatus.error:
                    self.connection_status = ConnectionStatus.disconnected
                async with self._state_cv:
                    self._state_cv.notify_all()
                if subscription_task is not None:
                    subscription_task.cancel()
                    with suppress(Exception):
                        await subscription_task
                    subscription_task = None
                with suppress(Exception):
                    await self._robot_client.logout()
            async with self._state_cv:
                self._state_cv.notify_all()
            
            await asyncio.sleep(0.5)
            self.connection_status = ConnectionStatus.idle
        
    async def _subscription_task(self, robot_client):
        while True:
            try:
                sub = robot_client.subscribe([
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.ControllerState, rws.SubscriptionResourcePriority.Medium),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.OperationalMode, rws.SubscriptionResourcePriority.Medium),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.ExecutionState, rws.SubscriptionResourcePriority.Medium),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.Elog, rws.SubscriptionResourcePriority.Medium),
                # SubscriptionResourceRequest(SubscriptionResourceType.PersVar, SubscriptionResourcePriority.Medium, 
                    # {"task": None, "name": "motion_program_state"}),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.Signal, rws.SubscriptionResourcePriority.High, 
                    "motion_program_current_cmd_num"),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.Signal, rws.SubscriptionResourcePriority.High, 
                    "motion_program_queued_cmd_num"),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.Signal, rws.SubscriptionResourcePriority.High, 
                    "motion_program_preempt_current"),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.Signal, rws.SubscriptionResourcePriority.High, 
                    "motion_program_seqno"),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.Signal, rws.SubscriptionResourcePriority.High, 
                    "motion_program_seqno_complete"),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.Signal, rws.SubscriptionResourcePriority.High, 
                    "motion_program_seqno_started"),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.Signal, rws.SubscriptionResourcePriority.High, 
                    "motion_program_executing"),
                rws.SubscriptionResourceRequest(rws.SubscriptionResourceType.Signal, rws.SubscriptionResourcePriority.High, 
                    "motion_program_error")
                ])

                async for data in sub:
                    async with self._state_cv:
                        if isinstance(data, rws.ControllerState):
                            self.controller_state = self._convert_controller_state(data.state)
                        elif isinstance(data, rws.OperationalMode):
                            self.controller_opmode = self._convert_opmode(data.mode)
                        elif isinstance(data, rws.RAPIDExecutionState):
                            self.exec_state = self._convert_execstate(data.ctrlexecstate)
                        elif isinstance(data, rws.Signal):
                            if data.name == "motion_program_executing":
                                self.exec_state = float(data.lvalue) > 0.
                            elif data.name == "motion_program_seqno":
                                self.exec_motion_program_seqno = int(float(data.lvalue))
                            elif data.name == "motion_program_seqno_started":
                                self.exec_motion_program_seqno_started = int(float(data.lvalue))
                            elif data.name == "motion_program_seqno_complete":
                                self.exec_motion_program_seqno_complete = int(float(data.lvalue))
                            elif data.name == "motion_program_current_cmd_num":
                                self.exec_current_cmd_num = int(float(data.lvalue))
                            elif data.name == "motion_program_queued_cmd_num":
                                self.exec_queued_cmd_num = int(float(data.lvalue))
                            elif data.name == "motion_program_preempt_current":
                                self.exec_current_preempt_num = int(float(data.lvalue))
                            elif data.name == "motion_program_egm_active":
                                self.exec_egm_active = float(data.lvalue) > 0.
                            elif data.name == "motion_program_error":
                                self.exec_error = float(data.lvalue) > 0.
                        self._state_cv.notify_all()
            except Exception:
                traceback.print_exc()
                raise   
            finally:
                with suppress(Exception):
                    await sub.aclose()

    async def _update_state(self):
        async with self._state_cv:
            self._state_cv.notify_all()
            self.controller_state = self._convert_controller_state(await self._robot_client.get_controller_state())
            self.controller_opmode = self._convert_opmode(await self._robot_client.get_operation_mode())
            self.exec_state = self._convert_execstate((await self._robot_client.get_execution_state()).ctrlexecstate)
            self.exec_error = int(await self._robot_client.get_digital_io("motion_program_error")) > 0
            self.exec_current_cmd_num = int(await self._robot_client.get_analog_io("motion_program_current_cmd_num"))
            self.exec_queued_cmd_num = int(await self._robot_client.get_analog_io("motion_program_queued_cmd_num"))
            self.exec_current_preempt_num = int(await self._robot_client.get_analog_io("motion_program_preempt_current"))
            self.exec_motion_program_seqno_complete = int(await self._robot_client.get_analog_io("motion_program_seqno_complete"))
            self.exec_motion_program_seqno_started = int(await self._robot_client.get_analog_io("motion_program_seqno_started"))
            self.exec_egm_active = int(await self._robot_client.get_analog_io("motion_program_egm_active"))
            self._last_contact = time.perf_counter()


    async def _clean_files(self):
        now = time.perf_counter()
        if now - self._last_clean < 10:
            return
        self._last_clean = now        
        
        try:
            del_files = []
            files = await self._robot_client.list_files(self._ramdisk)
            for f in files:
                f_match = self._file_clean_re.match(f)
                if f_match is not None:
                    f_seqno = int(f_match.group(1))
                    if f_seqno < self._mp_seqno_i - 5:
                        del_files.append(f)
            for f2 in del_files:
                await self._robot_client.delete_file(f"{self._ramdisk}/{f2}")
        except:
            traceback.print_exc()

    def _convert_opmode(self, abb_opmode):
        if abb_opmode == "MANR":
            return 1
        if abb_opmode == "MANF":
            return 2
        if abb_opmode == "AUTO":
            return 3
        return 0

    def _convert_controller_state(self, abb_state):
        if abb_state == "init":
            return 1
        elif abb_state == "motoron":
            return 2
        elif abb_state == "motoroff":
            return 3
        elif abb_state == "guardstop":
            return 4
        elif abb_state == "emergencystop":
            return 5
        elif abb_state == "emergencystopreset":
            return 6
        else:
            return 0

    def _convert_execstate(self, abb_state):
        return abb_state == "running"

    def execute_motion_program(self, motion_program, *, start_timeout = 0.05, running_state = None, 
        enable_motion_logging = True):

        status_queue = asyncio.Queue()

        async def run_motion_program():
            try:
                mp_gen = self._execute_motion_program_gen(motion_program, start_timeout=start_timeout, 
                    running_state=running_state, enable_motion_logging=enable_motion_logging)

                async for state, data in mp_gen:
                    await status_queue.put((state, data))
            except BaseException as e:
                await status_queue.put((MotionProgramState.error, e))
                raise

        mp_task = asyncio.create_task(run_motion_program())

        return mp_task, status_queue

    async def _execute_motion_program_gen(self, motion_program, *, start_timeout = 0.05, running_state = None, 
        enable_motion_logging = True):
        
        if (len(motion_program._commands) == 0):
            motion_program.WaitTime(1e-3)
        if running_state is None:
            running_state = MotionProgramState.running
            if isinstance(motion_program._commands[-1], abb_exec.egm_commands.EGMRunJointCommand):
                running_state = MotionProgramState.running_egm_joint_control
            elif isinstance(motion_program._commands[-1], abb_exec.egm_commands.EGMRunPoseCommand):
                running_state = MotionProgramState.running_egm_pose_control
            elif isinstance(motion_program._egm_config, abb_exec.EGMPathCorrectionConfig):
                running_state = MotionProgramState.running_egm_path_corr

        egm_stop = isinstance(motion_program._commands[-1], abb_exec.egm_commands.EGMRunJointCommand) \
                or isinstance(motion_program._commands[-1], abb_exec.egm_commands.EGMRunPoseCommand)

        self.motion_program_state == MotionProgramState.starting

        seqno = -1
        await asyncio.wait_for(self._motion_program_lock.acquire(), timeout = start_timeout)
        filename = None
        try:

            self._mp_seqno_i += 1
            seqno_started = int(await self._robot_client.get_analog_io("motion_program_seqno_started"))
            if seqno_started >= self._mp_seqno_i:
                self._mp_seqno_i = seqno_started + 1
            seqno = self._mp_seqno_i
        
            async with self._state_cv:
                await asyncio.wait_for(self._state_cv.wait_for(lambda: self.connection_status == ConnectionStatus.connected),0.1)
                assert self.connection_status == ConnectionStatus.connected, "Not connected to robot"
            self.motion_program_state == MotionProgramState.starting
            yield (MotionProgramState.starting, seqno)
            if enable_motion_logging:
                await self._robot_mp_client.enable_motion_logging()
            else:
                await self._robot_mp_client.disable_motion_logging()

            #log_seqnum = await self._robot_mp_client.execute_motion_program(motion_program, 
            #            wait = False, seqno = seqno)
            
            b = motion_program.get_program_bytes(seqno)
            assert len(b) > 0, "Motion program must not be empty"

            
            filename = f"{self._ramdisk}/motion_program---seqno-{seqno}.bin"
            await self._robot_client.upload_file(filename, b)

            assert self.exec_state and self.exec_motion_program_seqno == -1

            await self._robot_client.set_analog_io("motion_program_driver_abort", 0)
            await self._robot_client.set_analog_io("motion_program_seqno_command", seqno)

            await self._update_state()
            self.motion_program_state = running_state
            yield (running_state, (-1,-1,-1))
            cur_cmd_num = -1
            cur_queued_num = -1
            cur_preempt_num = -1
            sent_cmd_num = -1
            sent_queued_num = -1
            sent_preempt_num = -1
            try:
                last_update = time.perf_counter()
                while True:
                    with suppress(asyncio.TimeoutError):
                        async with self._state_cv:
                            if not self.exec_state or self.exec_motion_program_seqno_complete >= seqno:
                                break
                            await asyncio.wait_for(self._state_cv.wait_for(lambda: not self.exec_state or \
                                self.exec_motion_program_seqno_complete >= seqno or \
                                self.exec_current_cmd_num > sent_cmd_num or \
                                self.exec_queued_cmd_num > sent_queued_num or \
                                self.exec_current_preempt_num > sent_preempt_num),1)
                    cur_cmd_num = self.exec_current_cmd_num
                    cur_queued_num = self.exec_queued_cmd_num
                    cur_preempt_num = self.exec_current_preempt_num
                    
                    if (cur_cmd_num > sent_cmd_num) or (cur_queued_num > sent_queued_num) or \
                        (cur_preempt_num > sent_preempt_num) or time.perf_counter() - last_update > 2.5:
                        sent_cmd_num = cur_cmd_num
                        sent_queued_num = cur_queued_num
                        sent_preempt_num = cur_preempt_num
                        last_update = time.perf_counter()
                        yield (running_state, (sent_cmd_num, sent_queued_num, sent_preempt_num))
                
                
                res = None
                if float(await self._robot_client.get_digital_io("motion_program_error")) > 0.:
                    await asyncio.sleep(0.25)
                    err = Exception("ABB motion execution failed")
                    err_fname = f"{self._ramdisk}/motion_program_err---seqno-{seqno}.json"
                    try:
                        err_json_txt = await self._robot_client.read_file(err_fname)
                        err_json = json.loads(err_json_txt)
                        err_code = int(err_json["error_domain"]*10000+err_json['error_number'])
                        err = Exception(f"ABB motion execution failed with error code {err_code} " \
                            f"with message \"{err_json['error_title']}\"")
                        
                    except Exception:
                        traceback.print_exc()
                    finally:
                        with suppress(Exception):
                            await self._robot_client.delete_file(err_fname)
                    raise err

                if enable_motion_logging:
                    recording_fname = f"{self._ramdisk}/log-motion_program---seqno-{seqno}.bin"
                    recording_bin = await self._robot_client.read_file(recording_fname)
                    with suppress(Exception):
                        await self._robot_client.delete_file(recording_fname)
                    res = abb_exec.abb_motion_program_exec_client._unpack_motion_program_result_log(recording_bin)

                yield MotionProgramState.complete, res
            except asyncio.CancelledError:
                self.motion_program_state == MotionProgramState.stop_requested
                yield (MotionProgramState.stop_requested, None)
                if egm_stop:
                    self.motion_program_state = MotionProgramState.stopping_egm
                    await self._robot_mp_client.stop_egm()
                else:
                    self.motion_program_state = MotionProgramState.stopping
                    await self._robot_client.set_analog_io("motion_program_seqno_command", -1)
                    await self._robot_client.set_analog_io("motion_program_driver_abort", 1)
                yield (MotionProgramState.cancelled, None)
        
        finally:
            try:                
                with suppress(asyncio.TimeoutError):
                    async with self._state_cv:
                        await asyncio.wait_for(self._state_cv.wait_for(lambda: self.exec_motion_program_seqno == -1 \
                            or not self.exec_state),1)
                
                if filename is not None:
                    with suppress(Exception):
                        await self._robot_client.delete_file(filename)

                #assert self.exec_motion_program_seqno != seqno or not self.exec_state, "Error ending motion program"
            finally:
                self.motion_program_state = MotionProgramState.idle
                self._motion_program_lock.release()

    def start_joint_control(self, *, start_timeout = 0.05, enable_motion_logging = True, tool=None, payload=None,
        payload_pose=None):
        
        mm = abb_exec.egm_minmax(-1e-3,1e-3)

        egm_config = abb_exec.EGMJointTargetConfig(
            mm, mm, mm, mm, mm ,mm, 1000, 1000
        )

        mp_tool = None
        mp_payload = None

        if tool is not None:
            mp_tool = _motion_program_conv.rr_tool_to_abb(tool)

        if payload is not None:
            mp_payload = _motion_program_conv.rr_payload_to_abb(payload, payload_pose)

        # TODO: tool, payload, wobj
        mp = abb_exec.MotionProgram(egm_config = egm_config, tool=mp_tool, gripload=mp_payload)
        mp.EGMRunJoint(1e9, 0.005, 0.005)

        return self.execute_motion_program(mp, start_timeout = start_timeout,
            running_state = MotionProgramState.running_egm_joint_control, 
            enable_motion_logging = enable_motion_logging)

    async def preempt_motion_program(self, motion_program, preempt_number, preempt_cmdnum):
        seqno = int(await self._robot_client.get_analog_io("motion_program_seqno"))
        assert seqno > 0, "Motion program not running"
        b = motion_program.get_program_bytes(seqno)
        assert len(b) > 0, "Motion program must not be empty"

        
        filename = f"{self._ramdisk}/motion_program_p{preempt_number}---seqno-{seqno}.bin"
        await self._robot_client.upload_file(filename, b)

        await self._robot_client.set_analog_io("motion_program_preempt_cmd_num", preempt_cmdnum)
        await self._robot_client.set_analog_io("motion_program_preempt", preempt_number)

    def send_enable(self):
        if not self.exec_state:
            self._user_robot_client.set_controller_state("motoron")

    def send_disable(self):
        self._user_robot_client.set_controller_state("motoroff")

    def reset_errors(self):
        async def _reset_errors_task():
            async with self._state_cv:
                try:
                    await asyncio.wait_for(self._state_cv.wait_for(lambda: not self.exec_state),0.05)
                except asyncio.TimeoutError:
                    return

                await self._robot_client.resetpp()
                await self._robot_client.set_analog_io("motion_program_seqno_command", -1)
                await self._robot_client.set_analog_io("motion_program_seqno", -1)
                ctrl_state = await self._robot_client.get_controller_state()
                assert ctrl_state == "motoron", "Controller motors not enabled"
                await self._robot_client.start(cycle="forever")

        fut = asyncio.run_coroutine_threadsafe(_reset_errors_task(), self.loop)
        fut.result()


    def send_stop_all(self):
        self._user_robot_client.stop()


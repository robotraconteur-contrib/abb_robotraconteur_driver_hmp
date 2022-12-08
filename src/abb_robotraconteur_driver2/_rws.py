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
        self.exec_current_cmd_num = -1
        self.exec_queued_cmd_num = -1
        self.exec_egm_active = -1

        self.egm_communication_failure = False
        self._last_egm_reset = 0

        self._stop_all = False

        self._connect_task_fut = None

        self._state_cv = asyncio.Condition()
        self._motion_program_lock = asyncio.Lock()
        self.loop = None


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
                if self.exec_state:                    
                    await self._robot_client.stop()
                    await self._update_state()
                    assert self.exec_state is False

                subscription_task = asyncio.create_task(self._subscription_task(self._robot_client))
                
                await self._update_state()
                self.connection_status = ConnectionStatus.connected

                while True:
                    if self.egm_communication_failure:
                        await self._do_egm_start()
                    if time.perf_counter() - self._last_contact > 1:
                        await self._update_state()
                    await asyncio.sleep(1)
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
            with self._state_cv:
                self._state_cv.notify_all()
            
            await asyncio.sleep(0.5)
            self.connection_status = ConnectionStatus.idle
        
        
    async def _do_egm_start(self):
        if self.egm_communication_failure and not self.exec_state:
            now = time.perf_counter()
            if (now - self._last_egm_reset) > 1:                
                self._last_egm_reset = time.perf_counter()
                mp = abb_exec.MotionProgram()
                mp.WaitTime(0.001)

                #with suppress(Exception):
                try:
                    r = self._execute_motion_program_gen(mp, start_timeout = 0.01, enable_motion_logging=False)
                    async for _ in r:
                        pass
                except:
                    traceback.print_exc()

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
                    "motion_program_seqno"),
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
                            elif data.name == "motion_program_current_cmd_num":
                                self.exec_current_cmd_num = int(float(data.lvalue))
                            elif data.name == "motion_program_queued_cmd_num":
                                self.exec_queued_cmd_num = int(float(data.lvalue))
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
            self.exec_egm_active = int(await self._robot_client.get_analog_io("motion_program_egm_active"))
            self._last_contact = time.perf_counter()


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
        self._mp_seqno_i += 1
        seqno = self._mp_seqno_i

        self.motion_program_state == MotionProgramState.starting


        await asyncio.wait_for(self._motion_program_lock.acquire(), timeout = start_timeout)
        
        try:
            async with self._state_cv:
                await asyncio.wait_for(self._state_cv.wait_for(lambda: self.connection_status == ConnectionStatus.connected),0.1)
                assert self.connection_status == ConnectionStatus.connected, "Not connected to robot"
            self.motion_program_state == MotionProgramState.starting
            yield (MotionProgramState.starting, seqno)
            if enable_motion_logging:
                await self._robot_mp_client.enable_motion_logging()
            else:
                await self._robot_mp_client.disable_motion_logging()

            log_seqnum = await self._robot_mp_client.execute_motion_program(motion_program, 
                        wait = False, seqno = seqno)
            
            await self._update_state()
            self.motion_program_state = running_state
            yield (running_state, (-1,-1))
            cur_cmd_num = -1
            cur_queued_num = -1
            sent_cmd_num = -1
            sent_queued_num = -1
            try:
                last_update = time.perf_counter()
                while True:
                    with suppress(asyncio.TimeoutError):
                        async with self._state_cv:
                            if not self.exec_state:
                                break
                            await asyncio.wait_for(self._state_cv.wait_for(lambda: not self.exec_state or \
                                self.exec_current_cmd_num > sent_cmd_num or \
                                self.exec_queued_cmd_num > sent_queued_num),1)
                        cur_cmd_num = self.exec_current_cmd_num
                        cur_queued_num = self.exec_queued_cmd_num
                    
                    if (cur_cmd_num > sent_cmd_num) or (cur_queued_num > sent_queued_num) \
                        or time.perf_counter() - last_update > 2.5:
                        sent_cmd_num = cur_cmd_num
                        sent_queued_num = cur_queued_num
                        last_update = time.perf_counter()
                        yield (running_state, (sent_cmd_num, sent_queued_num))
                
                if enable_motion_logging:            
                    res = await self._robot_mp_client.read_motion_program_result_log(log_seqnum)
                else:
                    res = None
                    if float(await self._robot_client.get_digital_io("motion_program_error")) > 0.:
                        raise Exception("ABB motion execution failed")

                yield MotionProgramState.complete, res
            except asyncio.CancelledError:
                self.motion_program_state == MotionProgramState.stop_requested
                yield (MotionProgramState.stop_requested, None)
                if egm_stop:
                    self.motion_program_state = MotionProgramState.stopping_egm
                    await self._robot_mp_client.stop_egm()
                else:
                    self.motion_program_state = MotionProgramState.stopping
                    await self._robot_mp_client.stop_motion_program()
                yield (MotionProgramState.cancelled, None)
        
        finally:
            try:
                with suppress(asyncio.TimeoutError):
                    async with self._state_cv:
                        await asyncio.wait_for(self._state_cv.wait_for(lambda: self.exec_motion_program_seqno != seqno \
                            or not self.exec_state),0.5)
                
                assert self.exec_motion_program_seqno != seqno or not self.exec_state, "Error ending motion program"
            finally:
                self.motion_program_state = MotionProgramState.idle
                self._motion_program_lock.release()

    def start_joint_control(self, *, start_timeout = 0.05, enable_motion_logging = True):
        
        mm = abb_exec.egm_minmax(-1e-3,1e-3)

        egm_config = abb_exec.EGMJointTargetConfig(
            mm, mm, mm, mm, mm ,mm, 1000, 1000
        )

        # TODO: tool, payload, wobj
        mp = abb_exec.MotionProgram(egm_config = egm_config)
        mp.EGMRunJoint(1e9, 0.005, 0.005)

        return self.execute_motion_program(mp, start_timeout = start_timeout,
            running_state = MotionProgramState.running_egm_joint_control, 
            enable_motion_logging = enable_motion_logging)

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

            mp = abb_exec.MotionProgram()
            mp.WaitTime(0.001)

            #with suppress(Exception):
            try:
                r = self._execute_motion_program_gen(mp, start_timeout = 0.01, enable_motion_logging=False)
                async for _ in r:
                    pass
            except:
                traceback.print_exc()
        fut = asyncio.run_coroutine_threadsafe(_reset_errors_task(), self.loop)
        fut.result()

    def send_stop_all(self):
        self._user_robot_client.stop()

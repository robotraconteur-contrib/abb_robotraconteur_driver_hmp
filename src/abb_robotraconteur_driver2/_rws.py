import abb_robot_client.rws as rws
from typing import NamedTuple, Callable
import threading
from enum import IntEnum
import traceback
import abb_motion_program_exec as abb_exec
import time
import RobotRaconteur as RR
from contextlib import suppress

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

class _mp_request(NamedTuple):
    seqno: int
    mp: abb_exec.MotionProgram
    handler: Callable
    running_state: MotionProgramState
    egm_stop: bool
    enable_motion_log: bool

class ABBRobotRWSImpl:
    def __init__(self, robot_url, robot_username = None, robot_password = None):
        self._robot_url = robot_url
        self._robot_username = robot_username
        self._robot_password = robot_password
        self._keep_going = False
        self._thread_cv_wake = False
        self._thread_cv = threading.Condition()
        self._thread = None
        self.connection_status = ConnectionStatus.idle
        self._subscription = None
        self._last_error = None
        self._robot_client = None
        self._user_robot_client = rws.RWS(robot_url, robot_username, robot_password)
        self._robot_mp_client = None

        self.controller_opmode: int = 0
        self.controller_state: int = 0
        self.exec_state: bool = False
        self.exec_error: bool = False
        self.motion_program_state = MotionProgramState.idle

        self._motion_programs_req = []
        self._current_motion_program_req = None
        self._current_motion_program_start_time = 0
        self._current_motion_program_stop_time = 0
        self._current_motion_program_log_seqnum = 0

        self._mp_seqno_i = 1

        self._mp_stop_req = set()

        self._last_contact = time.perf_counter()
        self.exec_motion_program_seqno = -1
        self.exec_current_cmd_num = -1
        self.exec_queued_cmd_num = -1
        self.exec_egm_active = -1

        self.egm_communication_failure = False
        self._last_egm_reset = 0

    def _run(self):
        while True:
            
            if self.connection_status != ConnectionStatus.connected:
                self._do_connect()
            
            if self.connection_status == ConnectionStatus.connected:
                self._do_egm_start()

                self._do_motion_program()

                #if time.perf_counter() - self._last_contact > 1:
                self._do_rws_check()

            
            wait_timeout = 0.1
            if self.connection_status == ConnectionStatus.connected and not self.egm_communication_failure:
                wait_timeout = 1

            with self._thread_cv:
                if not self._keep_going: break
                if self._thread_cv_wake:
                    self._thread_cv_wake = False
                else:
                    self._thread_cv.wait(timeout=wait_timeout)
                    if not self._keep_going: break

        self._do_thread_fini()

    def _do_connect(self):
        try:                
            try:
                s = self._subscription
                self._subscription = None
                if s is not None:
                    s.close()
            except:
                traceback.print_exc()

            if self._robot_client is None:
                self._robot_client = rws.RWS(self._robot_url, self._robot_username, self._robot_password)
                self._robot_mp_client = abb_exec.MotionProgramExecClient(abb_client = self._robot_client)

            self.connection_status = ConnectionStatus.connecting
            self._update_state()
            if self.exec_state:
                with suppress(Exception):
                    self._robot_client.stop()

            sub = self._robot_client.subscribe([
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
            ], self._subscription_handler)
            self._subscription = sub
            self._update_state()
            self.connection_status = ConnectionStatus.connected
        except Exception as e:
            self._handle_error(e)

    def _do_rws_check(self):
        try:
            self._update_state()
        except Exception as e:
            self._handle_error(e)

    def _do_motion_program(self):
        try:
            with self._thread_cv:
                if self._current_motion_program_req is not None:
                    try:
                        if self.exec_state:
                            if self.motion_program_state == MotionProgramState.starting:
                                if self.exec_motion_program_seqno == self._current_motion_program_req.seqno:
                                    self.motion_program_state = self._current_motion_program_req.running_state
                                    self._call_mp_handler(self._current_motion_program_req.handler, 
                                        self._current_motion_program_req, self.motion_program_state)
                                elif time.perf_counter() - self._current_motion_program_start_time > 2:
                                    raise Exception("Failed to start motion program")

                            if self.motion_program_state in (MotionProgramState.running, 
                                MotionProgramState.running_egm_pose_control, 
                                MotionProgramState.running_egm_joint_control,
                                MotionProgramState.running_egm_path_corr):
                                if self._current_motion_program_req.seqno in self._mp_stop_req:
                                    self._mp_stop_req.remove(self._current_motion_program_req.seqno)
                                    self.motion_program_state = MotionProgramState.stop_requested

                            if self.motion_program_state == MotionProgramState.stop_requested:
                                self._current_motion_program_stop_time = time.perf_counter()
                                if self._current_motion_program_req.egm_stop:
                                    self.motion_program_state = MotionProgramState.stopping_egm
                                    self._robot_mp_client.stop_egm()
                                else:
                                    self.motion_program_state = MotionProgramState.stopping
                                    self._robot_mp_client.stop_motion_program()
                                self._call_mp_handler(self._current_motion_program_req.handler, 
                                        self._current_motion_program_req, self.motion_program_state)
                            

                            if self.motion_program_state in (MotionProgramState.stopping_egm, MotionProgramState.stopping):
                                if time.perf_counter() - self._current_motion_program_stop_time > 2:
                                    raise Exception("Failed to stop motion program")
                        else:
                            cur_req = self._current_motion_program_req
                            if cur_req.enable_motion_log:
                                res = self._robot_mp_client.read_motion_program_result_log(self._current_motion_program_log_seqnum)
                            else:
                                res = None
                                if float(self._robot_client.get_digital_io("motion_program_error")) > 0.:
                                    raise Exception("ABB motion execution failed")
                            self._current_motion_program_req = None
                            self.motion_program_state = MotionProgramState.idle
                            self._call_mp_handler(cur_req.handler, cur_req,
                                MotionProgramState.complete, res)
                    except Exception as e:
                        cur_req = self._current_motion_program_req
                        self._current_motion_program_req = None
                        self.motion_program_state = MotionProgramState.error
                        traceback.print_exc()                    
                        self._call_mp_handler(cur_req.handler, cur_req, 
                                MotionProgramState.error, e)
                        # TODO: physical e-stop?
                        if self.exec_state:
                            with suppress(Exception):
                                self._robot_client.stop()
                            with suppress(Exception):
                                self.exec_state = self._robot_client.get_execution_state()
                else:
                    if self.exec_state:
                        self.motion_program_state = MotionProgramState.error
                        with suppress(Exception):
                            self._robot_client.stop()
                            self.exec_state = self._robot_client.get_execution_state()
                    else:
                        self.motion_program_state = MotionProgramState.idle
                        if len(self._motion_programs_req) > 0:
                            next_mp_req : _mp_request = self._motion_programs_req.pop(0)
                            self._mp_stop_req = set(filter(lambda x: x>=next_mp_req.seqno, self._mp_stop_req))
                            if next_mp_req.seqno in self._mp_stop_req:
                                self._mp_stop_req.remove(next_mp_req.seqno)
                                self._call_mp_handler(next_mp_req.handler, next_mp_req, MotionProgramState.error, \
                                    Exception("Motion program cancelled"))
                                self._wake()
                                return
                            try:
                                assert not self.exec_state
                                self._current_motion_program_req = next_mp_req
                                self.motion_program_state = MotionProgramState.starting
                                self._call_mp_handler(next_mp_req.handler, next_mp_req,
                                    MotionProgramState.starting, next_mp_req.seqno)
                                if next_mp_req.enable_motion_log:
                                    self._robot_mp_client.enable_motion_logging()
                                else:
                                    self._robot_mp_client.disable_motion_logging()
                                self._current_motion_program_log_seqnum = \
                                    self._robot_mp_client.execute_motion_program(next_mp_req.mp, wait = False, \
                                        seqno = next_mp_req.seqno)
                                self._current_motion_program_start_time = time.perf_counter()
                                self._current_motion_program_stop_time = 0
                                self.exec_state = self._convert_execstate(self._robot_client.get_execution_state().ctrlexecstate)
                                self._wake()
                            except Exception as e:                         
                                self.motion_program_state = MotionProgramState.error
                                self._call_mp_handler(next_mp_req.handler, next_mp_req, MotionProgramState.error, e)
                                traceback.print_exc()
                                with suppress(Exception):
                                    self._robot_client.stop()                            

                self._last_contact = time.perf_counter()
        except Exception as e:
            self._handle_error(e)

    def _update_state(self):
        self.controller_state = self._convert_controller_state(self._robot_client.get_controller_state())
        self.controller_opmode = self._convert_opmode(self._robot_client.get_operation_mode())
        self.exec_state = self._convert_execstate(self._robot_client.get_execution_state().ctrlexecstate)
        self.exec_error = int(self._robot_client.get_digital_io("motion_program_error")) > 0
        self.exec_current_cmd_num = int(self._robot_client.get_analog_io("motion_program_current_cmd_num"))
        self.exec_queued_cmd_num = int(self._robot_client.get_analog_io("motion_program_queued_cmd_num"))
        self.exec_egm_active = int(self._robot_client.get_analog_io("motion_program_egm_active"))
        self._last_contact = time.perf_counter()

    def _clear_state(self):
        self.controller_state = 0
        self.controller_opmode = 0
        self.exec_state = False

    def _handle_error(self, err):
        self._clear_state()
        if self._keep_going:
            traceback.print_exc()
        self._last_error = err
        self.connection_status = ConnectionStatus.error

    def _do_egm_start(self):
        if self.egm_communication_failure and self._current_motion_program_req is None \
            and len(self._motion_programs_req) == 0 and not self.exec_state:
            now = time.perf_counter()
            if (now - self._last_egm_reset) > 1:                
                self._last_egm_reset = time.perf_counter()
                req = self._get_start_egm_streaming_req()
                self._motion_programs_req.append(req)


    def _do_thread_fini(self):
        with suppress(Exception):
            if self._robot_client.get_execution_state().ctrlexecstate == "running":
                self._robot_client.stop()

        with suppress(Exception):
            self._robot_client.logout()

    def start(self):
        self._keep_going = True
        self._thread = threading.Thread(target = self._run)
        self._thread.daemon = True
        self._thread.start()

    def close(self):
        with self._thread_cv:
            self._keep_going = False
            self._wake()
        if self._thread:
            self._thread.join()
        try:
            if self._subscription:
                self._subscription.close()
        except:
            traceback.print_exc()
        self._subscription = None

        with suppress(Exception):
            self._user_robot_client.logout()

        with suppress(Exception):
            self._thread.join(timeout = 5)

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

    def _subscription_handler(self, data):
        with self._thread_cv:
            self._wake()
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


    def start_joint_control(self, handler, enable_motion_logging = True):
        
        mm = abb_exec.egm_minmax(-1e-3,1e-3)

        egm_config = abb_exec.EGMJointTargetConfig(
            mm, mm, mm, mm, mm ,mm, 1000, 1000
        )

        # TODO: tool, payload, wobj
        mp = abb_exec.MotionProgram(egm_config = egm_config)
        mp.EGMRunJoint(1e9, 0.005, 0.005)

        with self._thread_cv:
            return self._execute_motion_program_nolock(mp, handler, MotionProgramState.running_egm_joint_control, enable_motion_logging)

    def execute_motion_program(self, motion_program, handler, running_state = None, enable_motion_logging = True):
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

        with self._thread_cv:
            return self._execute_motion_program_nolock(motion_program,handler,running_state, enable_motion_logging)

    def stop_motion_program(self, motion_program_req):
        with self._thread_cv:
            self._mp_stop_req.add(motion_program_req.seqno)
    
    def _execute_motion_program_nolock(self, motion_program, handler, running_state, enable_motion_logging):
        if (self.motion_program_state != MotionProgramState.idle and \
            self.motion_program_state != MotionProgramState.error) or len(self._motion_programs_req) > 0:
            raise RR.InvalidOperationException("Motion program already executing")
        self._mp_seqno_i += 1            
        egm_stop = isinstance(motion_program._commands[-1], abb_exec.egm_commands.EGMRunJointCommand) \
            or isinstance(motion_program._commands[-1], abb_exec.egm_commands.EGMRunPoseCommand)
            
        req = _mp_request(self._mp_seqno_i, motion_program, handler, running_state, egm_stop, enable_motion_logging)
        self._motion_programs_req.append(req)

        return req

    def _call_mp_handler(self, handler, motion_program_req, state, param = None):
        if handler is None:
            return
        try:
            handler(motion_program_req, state, param)
        except:
            traceback.print_exc()

    def _get_start_egm_streaming_req(self):
        mp = abb_exec.MotionProgram()
        mp.WaitTime(0.001)

        self._mp_seqno_i += 1
        req = _mp_request(self._mp_seqno_i, mp, None, MotionProgramState.running, False, False)
        return req

    def _wake(self):
        self._thread_cv_wake = False
        self._thread_cv.notify()

    def is_motion_program_running(self, motion_program_req):
        r = self._current_motion_program_req
        if r is None:
            return False
        return self.motion_program_state == motion_program_req.running_state \
            and r.seqno == motion_program_req.seqno

    def send_enable(self):
        if not self.exec_state:
            self._user_robot_client.set_controller_state("motoron")

    def send_disable(self):
        self._user_robot_client.set_controller_state("motoroff")

    def reset_errors(self):
        if not self.exec_state:
            self._user_robot_client.resetpp()
    
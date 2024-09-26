import os
import time
import enum
import multiprocessing as mp
from multiprocessing.managers import SharedMemoryManager
import numpy as np

from umi.common.pose_util import adapt4fr
from umi.shared_memory.shared_memory_queue import (
    SharedMemoryQueue, Empty)
from umi.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer
from umi.common.pose_trajectory_interpolator import PoseTrajectoryInterpolator
from diffusion_policy.common.precise_sleep import precise_wait
from fairino import Robot

class Command(enum.Enum):
    STOP = 0
    SERVOL = 1
    SCHEDULE_WAYPOINT = 2


class FrInterpolationController(mp.Process):
    """
    To ensure sending command to the robot with predictable latency
    this controller need its separate process (due to python GIL)
    """


    def __init__(self,
            shm_manager: SharedMemoryManager, 
            robot_ip='192.168.58.2',
            frequency=125, 
            lookahead_time=0.1, 
            gain=300,
            max_pos_speed=0.25, # 5% of max speed
            max_rot_speed=0.16, # 5% of max speed
            launch_timeout=3,
            tcp_offset_pose=None,
            payload_mass=None,
            payload_com=None,
            joints_init=None,
            joints_init_speed=1.05,
            soft_real_time=False,
            verbose=False,
            receive_keys=None,
            get_max_k=None,
            receive_latency=0.0
            ):
        """
        frequency: CB2=125, UR3e=500
        lookahead_time: [0.03, 0.2]s smoothens the trajectory with this lookahead time
        gain: [100, 2000] proportional gain for following target position
        max_pos_speed: m/s
        max_rot_speed: rad/s
        tcp_offset_pose: 6d pose, 单位[mm][°]
        payload_mass: float
        payload_com: 3d position, center of mass
        soft_real_time: enables round-robin scheduling and real-time priority
            requires running scripts/rtprio_setup.sh before hand.

        """
        # verify
        assert 0 < frequency <= 500
        assert 0.03 <= lookahead_time <= 0.2
        assert 100 <= gain <= 2000
        assert 0 < max_pos_speed
        assert 0 < max_rot_speed
        if tcp_offset_pose is not None:
            # tcp_offset_pose = np.array(tcp_offset_pose)
            # assert tcp_offset_pose.shape == (6,)
            assert len(tcp_offset_pose) == 6
        if payload_mass is not None:
            assert 0 <= payload_mass <= 5
        if payload_com is not None:
            # payload_com = np.array(payload_com)
            # assert payload_com.shape == (3,)
            # assert payload_mass is not None
            assert len(payload_com) == 3
        if joints_init is not None:
            # joints_init = np.array(joints_init)
            assert len(joints_init) == 6

        super().__init__(name="FrInterpolationController")
        self.robot_ip = robot_ip
        self.frequency = frequency
        self.lookahead_time = lookahead_time
        self.gain = gain
        self.max_pos_speed = max_pos_speed
        self.max_rot_speed = max_rot_speed
        self.launch_timeout = launch_timeout
        self.tcp_offset_pose = tcp_offset_pose
        self.payload_mass = payload_mass
        self.payload_com = payload_com
        self.joints_init = joints_init
        self.joints_init_speed = joints_init_speed
        self.soft_real_time = soft_real_time
        self.receive_latency = receive_latency
        self.verbose = verbose
        self.tool_id = 1

        if get_max_k is None:
            get_max_k = int(frequency * 5)

        # build input queue
        example = {
            'cmd': Command.SERVOL.value,
            'target_pose': np.zeros((6,), dtype=np.float64),
            'duration': 0.0,
            'target_time': 0.0
        }
        input_queue = SharedMemoryQueue.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            buffer_size=256
        )

        # build ring buffer
        if receive_keys is None:
            receive_keys = [
                'ActualTCPPose',
                # 'ActualTCPSpeed',
                # 'ActualQ',
                # 'ActualQd',

                # 'TargetTCPPose',
                # 'TargetTCPSpeed',
                # 'TargetQ',
                # 'TargetQd'
            ]
        self.robot = Robot.RPC(self.robot_ip)
        example = dict()
        for key in receive_keys:
            # 一般返回二元组(ret_code, ret)，取第二个
            _, ret = getattr(self.robot, 'Get'+key)()
            if 'Pose' in key:
                ret = adapt4fr(ret, toFr=False)
            example[key] = np.array(ret)
        example['robot_receive_timestamp'] = time.time()
        example['robot_timestamp'] = time.time()
        ring_buffer = SharedMemoryRingBuffer.create_from_examples(
            shm_manager=shm_manager,
            examples=example,
            get_max_k=get_max_k,
            get_time_budget=0.2,
            put_desired_frequency=frequency
        )

        self.ready_event = mp.Event()
        self.input_queue = input_queue
        self.ring_buffer = ring_buffer
        self.receive_keys = receive_keys
    
    # ========= launch method ===========
    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        if self.verbose:
            print(f"[FrInterpolationController] Controller process spawned at {self.pid}")

    def stop(self, wait=True):
        message = {
            'cmd': Command.STOP.value
        }
        self.input_queue.put(message)
        if wait:
            self.stop_wait()

    def start_wait(self):
        self.ready_event.wait(self.launch_timeout)
        assert self.is_alive()
    
    def stop_wait(self):
        self.join()
    
    @property
    def is_ready(self):
        return self.ready_event.is_set()

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        
    # ========= command methods ============
    def servoL(self, pose, duration=0.1):
        """
        duration: desired time to reach pose
        """
        assert self.is_alive()
        assert(duration >= (1/self.frequency))
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SERVOL.value,
            'target_pose': pose,
            'duration': duration
        }
        self.input_queue.put(message)
    
    def schedule_waypoint(self, pose, target_time):
        pose = np.array(pose)
        assert pose.shape == (6,)

        message = {
            'cmd': Command.SCHEDULE_WAYPOINT.value,
            'target_pose': pose,
            'target_time': target_time
        }
        self.input_queue.put(message)

    # ========= receive APIs =============
    def get_state(self, k=None, out=None):
        if k is None:
            return self.ring_buffer.get(out=out)
        else:
            return self.ring_buffer.get_last_k(k=k,out=out)
    
    def get_all_state(self):
        return self.ring_buffer.get_all()
    
    # ========= main loop in process ============
    def run(self):
        # enable soft real-time
        if self.soft_real_time:
            os.sched_setscheduler(
                0, os.SCHED_RR, os.sched_param(20))

        # start fr
        robot = self.robot
        try:
            if self.verbose:
                print(f"[FrInterpolationController] Connect to robot: {self.robot_ip}")

            # set parameters
            if self.tcp_offset_pose is not None:
                '''
                id:坐标系编号，范围[0~14]；
                t_coord:工具中心点相对末端法兰中心位姿，单位[mm][°]；
                type:0-工具坐标系，1-传感器坐标系；
                install:安装位置，0-机器人末端，1-机器人外部
                '''
                assert robot.SetToolCoord(id=self.tool_id, t_coord=self.tcp_offset_pose, type=0, install=0) == 0

            if self.payload_mass is not None:
                # 单位[kg]
                assert robot.SetLoadWeight(self.payload_mass) == 0

            if self.payload_com is not None:
                # x,y,z:质心坐标，单位[mm]
                assert robot.SetLoadCoord(self.payload_com[0], self.payload_com[1], self.payload_com[2]) == 0
            
            # init pose
            print('init robot start', self.joints_init)
            if self.joints_init is not None:
                '''
                joint_pos:目标关节位置，单位[°]；
                tool:工具号，[0~14]；
                user:工件号，[0~14]；
                '''
                assert robot.MoveJ(self.joints_init, self.tool_id, 0) == 0
                print('init robot end', self.joints_init)

            # main loop
            dt = 1. / self.frequency
            dt = 1.0
            error, curr_pose = robot.GetActualTCPPose()
            assert error == 0
            curr_pose = adapt4fr(curr_pose, toFr=False)
            # use monotonic time to make sure the control loop never go backward
            curr_t = time.monotonic()
            last_waypoint_time = curr_t
            pose_interp = PoseTrajectoryInterpolator(
                times=[curr_t],
                poses=[curr_pose]
            )

            error = robot.ServoMoveStart()  # 伺服运动开始
            print("伺服运动开始错误码：", error)
            t_start = time.monotonic()
            iter_idx = 0
            keep_running = True
            while keep_running:
                # start control iteration
                # t_start = rtde_c.initPeriod()

                # send command to robot
                t_now = time.monotonic()
                # diff = t_now - pose_interp.times[-1]
                # if diff > 0:
                #     print('extrapolate', diff)
                pose_command = pose_interp(t_now)
                # 法奥位姿里的位置单位是毫米，这里需要转换
                pose_command = list(pose_command)
                pose_command = adapt4fr(pose_command)
                vel = 30
                acc = 50
                error = robot.ServoCart(mode=0,
                                       desc_pos=pose_command,
                                       vel=vel, acc=acc,
                                       cmdT=dt,
                                       filterT=self.lookahead_time, gain=self.gain)
                # print('t_now:', t_now, 'error:', error, 'pose:', pose_command)
                if error != 0:
                    print('error:', error, 'pose:', pose_command)
                assert error == 0
                # update robot state
                state = dict()
                for key in self.receive_keys:
                    _, ret = getattr(robot, 'Get' + key)()
                    if 'Pose' in key:
                        ret = adapt4fr(ret, toFr=False)
                    state[key] = np.array(ret)
                t_recv = time.time()
                state['robot_receive_timestamp'] = t_recv
                state['robot_timestamp'] = t_recv - self.receive_latency
                self.ring_buffer.put(state)

                # fetch command from queue
                try:
                    # commands = self.input_queue.get_all()
                    # n_cmd = len(commands['cmd'])
                    # process at most 1 command per cycle to maintain frequency
                    commands = self.input_queue.get_k(1)
                    n_cmd = len(commands['cmd'])
                except Empty:
                    n_cmd = 0

                # execute commands
                for i in range(n_cmd):
                    command = dict()
                    for key, value in commands.items():
                        command[key] = value[i]
                    cmd = command['cmd']

                    if cmd == Command.STOP.value:
                        keep_running = False
                        # stop immediately, ignore later commands
                        break
                    elif cmd == Command.SERVOL.value:
                        # since curr_pose always lag behind curr_target_pose
                        # if we start the next interpolation with curr_pose
                        # the command robot receive will have discontinouity 
                        # and cause jittery robot behavior.
                        target_pose = command['target_pose']
                        duration = float(command['duration'])
                        curr_time = t_now + dt
                        t_insert = curr_time + duration
                        pose_interp = pose_interp.drive_to_waypoint(
                            pose=target_pose,
                            time=t_insert,
                            curr_time=curr_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed
                        )
                        last_waypoint_time = t_insert
                        if self.verbose:
                            print("[FrInterpolationController] New pose target:{} duration:{}s".format(
                                target_pose, duration))
                    elif cmd == Command.SCHEDULE_WAYPOINT.value:
                        target_pose = command['target_pose']
                        target_time = float(command['target_time'])
                        # translate global time to monotonic time
                        target_time = time.monotonic() - time.time() + target_time
                        curr_time = t_now + dt
                        pose_interp = pose_interp.schedule_waypoint(
                            pose=target_pose,
                            time=target_time,
                            max_pos_speed=self.max_pos_speed,
                            max_rot_speed=self.max_rot_speed,
                            curr_time=curr_time,
                            last_waypoint_time=last_waypoint_time
                        )
                        last_waypoint_time = target_time
                    else:
                        keep_running = False
                        break

                # regulate frequency
                # rtde_c.waitPeriod(t_start)
                t_wait_util = t_start + (iter_idx + 1) * dt
                precise_wait(t_wait_util, time_func=time.monotonic)

                # first loop successful, ready to receive command
                if iter_idx == 0:
                    self.ready_event.set()
                iter_idx += 1

                if self.verbose:
                    print(f"[FrInterpolationController] Actual frequency {1/(time.monotonic() - t_now)}")
        except Exception as e:
            print('fr control error:', e)
        finally:
            # manditory cleanup
            # decelerate
            error = robot.ServoMoveEnd()  # 伺服运动结束
            print("伺服运动结束错误码：", error)

            # terminate
            self.ready_event.set()

            if self.verbose:
                print(f"[FrInterpolationController] Disconnected from robot: {self.robot_ip}")

import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from umi.real_world.dh_binary_driver import DHBinaryDriver
import time

def test():
    hostname = '192.168.58.18'
    port = 8887
    driver = DHBinaryDriver(hostname, port)
    initstate = 0
    force = 20
    # Define positions (in mm) and their corresponding speeds

    delay_between_positions = 2  # Fixed delay time (seconds)

    driver.open(hostname, port)
    print('===============================')
    print('=====开始初始化=====')
    # self.Initializationdirection()
    driver.Initialization()
    time.sleep(1)

    while initstate != 1:
        initstate = driver.GetInitState()
        print('initstate',initstate)
        time.sleep(1)
    print('=====夹爪初始化完成=====')
    print('===============================')

    driver.SetTargetForce(force)
    print('force = ', force)
    print('===============================')

    pos = 0
    vel = 20
    # self.SetTargetPosition(pos)
    # self.SetTargetSpeed(vel)
    driver.script_position_pd(pos, vel)

    position_and_velocitys = [(0, 20), (20, 40), (40, 40), (60, 20), (80, 20)]
    for position, velocity in position_and_velocitys:
        g_state = 0
        pos, vel = driver.pre_position(position,velocity)
        # self.script_position_pd(position,velocity)
        driver.SetTargetSpeed(int(vel))
        # Convert mm position to percentage (0-1000)
        position_percent = int((pos / 80.0) * 1000)
        driver.SetTargetPosition(position_percent)
        # while g_state == 0:
        #     g_state = self.GetGripState()
        #     sleep(0.2)
        time.sleep(1e-2)
        g_position = driver.GetCurrentPosition()
        print('g_position (percent)', g_position)
        print('g_position (mm)', g_position * 80 / 1000)
        g_velocity = driver.GetCurrentTargetSpeed()
        print('g_velocity', g_velocity)
        print('gripper_state', g_state)
        print('===============================')
        time.sleep(delay_between_positions)

    for position, velocity in sorted(position_and_velocitys, lambda x: x[0], reverse=True):
        g_state = 0
        # 使用 pre_position 方法
        driver.script_position_pd(position, velocity, force)
        # 等待夹爪动作完成
        # while g_state == 0:
        #     g_state = self.GetGripState()
        #     sleep(0.43)
        time.sleep(2e-2)
        g_position = driver.GetCurrentPosition()
        print('g_position (percent)', g_position)
        print('g_position (mm)', g_position * 80 / 1000)
        g_velocity = driver.GetCurrentTargetSpeed()
        print('g_velocity', g_velocity)
        print('gripper_state', g_state)
        print('===============================')
        time.sleep(delay_between_positions)

    driver.close()

if __name__ == '__main__':
    test()
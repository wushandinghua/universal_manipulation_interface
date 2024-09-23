import sys
import glob
import socket
from time import sleep
# from config import position_and_velocitys

# 创建TCP套接字
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


class DHBinaryDriver:
    gripper_ID = 0x01

    def __init__(self, hostname='192.168.1.21', port=15000):
        self.hostname = hostname
        self.port = port

    # ================= tcp/ip连接和关闭 ================
    def start(self):
        ret = -1
        ret = client_socket.connect_ex((self.hostname, self.port))
        print('Connect recv: ', ret)
        if ret < 0:
            print('Connect Error')
            ret = -1
        else:
            print('Connect Success')
            ret = 0
        return ret

    def close(self):
        client_socket.close()
        return True

    def device_write(self, nDate):
        length = 0
        date = bytes(nDate)
        length = client_socket.send(date)
        return length

    def device_read(self, length):
        date = client_socket.recv(length)
        return date

    # ================= 上下文管理器 ================
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    # ================= low level API:夹爪寄存器控制 ================
    def CRC16(self, nData, wLength):
        if nData == 0x00:
            return 0x0000
        wCRCWord = 0xFFFF
        poly = 0xA001
        for num in range(wLength):
            date = nData[num]
            wCRCWord = (date & 0xFF) ^ wCRCWord
            for bit in range(8):
                if (wCRCWord & 0x01) != 0:
                    wCRCWord >>= 1
                    wCRCWord ^= poly
                else:
                    wCRCWord >>= 1
        return wCRCWord

    def open(self,hostname, port):
        ret = self.start()
        if ret < 0:
            print('open failed')
            return ret
        else:
            print('open successful')
            return ret

    def WriteRegisterFunc(self, index, value):
        send_buf = [0, 0, 0, 0, 0, 0, 0, 0]
        send_buf[0] = self.gripper_ID  # 抓手ID
        send_buf[1] = 0x06  # 功能码（假设为写单个寄存器）
        send_buf[2] = (index >> 8) & 0xFF  # 寄存器地址的高字节
        send_buf[3] = index & 0xFF  # 寄存器地址的低字节
        send_buf[4] = (value >> 8) & 0xFF  # 要写入的值的高字节
        send_buf[5] = value & 0xFF  # 要写入的值的低字节

        crc = self.CRC16(send_buf, len(send_buf) - 2)  # 计算CRC校验
        send_buf[6] = crc & 0xFF  # CRC校验的低字节
        send_buf[7] = (crc >> 8) & 0xFF  # CRC校验的高字节

        # print(f"Send buffer before writing: {send_buf}")

        send_temp = send_buf
        ret = False
        retrycount = 3

        while (ret == False):
            ret = False

            if retrycount < 0:
                break
            retrycount = retrycount - 1

            wdlen = self.device_write(send_temp)
            if len(send_temp) != wdlen:
                print('write error! write: ', send_temp)
                continue

            rev_buf = self.device_read(8)
            if len(rev_buf) == wdlen:
                ret = True
        return ret

    def ReadRegisterFunc(self, index):
        send_buf = [0, 0, 0, 0, 0, 0, 0, 0]
        send_buf[0] = self.gripper_ID
        send_buf[1] = 0x03
        send_buf[2] = (index >> 8) & 0xFF
        send_buf[3] = index & 0xFF
        send_buf[4] = 0x00
        send_buf[5] = 0x01

        crc = self.CRC16(send_buf, len(send_buf) - 2)
        send_buf[6] = crc & 0xFF
        send_buf[7] = (crc >> 8) & 0xFF

        send_temp = send_buf
        ret = False
        retrycount = 3

        while (ret == False):
            ret = False

            if (retrycount < 0):
                break
            retrycount = retrycount - 1

            wdlen = self.device_write(send_temp)
            if len(send_temp) != wdlen:
                print('write error! write: ', send_temp)
                continue

            rev_buf = self.device_read(7)
            if (len(rev_buf) == 7) :
                value = ((rev_buf[4] & 0xFF) | (rev_buf[3] << 8))
                ret = True
            # ('read value : ', value)
        return value

    # =============== high level API ===============

    # 初始化夹爪
    def Initialization(self):
        # 每次上电后需要重新标定，这里简化到每次建立tcp连接后初始化，会有些浪费，但是不会有副作用
        self.WriteRegisterFunc(0x0100, 0xA5)

    # 初始化夹爪方向：0x00为夹爪会运行到最大的张开位置，并作为初始起点
    #                0x01为夹爪会运行到最小的闭合位置，并作为初始起点
    def Initializationdirection(self):
        self.WriteRegisterFunc(0x0300, 0x00)

    # 设置夹爪位置
    def SetTargetPosition(self, refpos):
        self.WriteRegisterFunc(0x0103, refpos)

    # 设置夹爪力矩
    def SetTargetForce(self, force):
        self.WriteRegisterFunc(0x0101, force)

    # 设置夹爪速度
    def SetTargetSpeed(self, velocity):
        self.WriteRegisterFunc(0x0104, velocity)

    # 获取位置和速度
    def pre_position(self, position: float, velocity: float):
        return position,velocity
    # 获取当前位置
    def GetCurrentPosition(self):
        return self.ReadRegisterFunc(0x0202)

    # 获取当前力矩
    def GetCurrentTargetForce(self):
        return self.ReadRegisterFunc(0x0101)

    # 获取当前速度
    def GetCurrentTargetSpeed(self):
        return self.ReadRegisterFunc(0x0104)

    # 获取初始化状态
    def GetInitState(self):
        return self.ReadRegisterFunc(0x0200)

    # 获取夹爪状态
    def GetGripState(self):
        return self.ReadRegisterFunc(0x0201)

    def custom_script(self, *args):
        state = self.GetGripState()
        position = self.GetCurrentPosition()
        position = (position * 80 / 1000)
        velocity = self.GetCurrentTargetSpeed()
        info = {
            'state': state,
            'position': position,
            'velocity': velocity
        }
        return info

    def script_position_pd(self, position: float, velocity: float, force: float=50.0):
        self.SetTargetForce(int(force))
        self.SetTargetSpeed(int(velocity))
        # Convert mm position to percentage (0-1000)
        position_percent = int(((position / 80.0) * 1000))
        # print('position_percent',position_percent)
        self.SetTargetPosition(position_percent)
        # time.sleep(1e-3)
        return self.custom_script(position, velocity)

    # ================= 测试代码 ================
    def socket_gripper(self):
        hostanme = '192.168.58.18'
        port = 8887
        initstate = 0
        force = 20
        # Define positions (in mm) and their corresponding speeds

        delay_between_positions = 2  # Fixed delay time (seconds)

        self.open(hostanme, port)
        print('===============================')
        print('=====开始初始化=====')
        # self.Initializationdirection()
        self.Initialization()
        sleep(1)

        while initstate != 1:
            initstate = self.GetInitState()
            print('initstate',initstate)
            sleep(1)
        print('=====夹爪初始化完成=====')
        print('===============================')

        self.SetTargetForce(force)
        print('force = ',force)
        print('===============================')

        pos = 80
        vel = 20
        # self.SetTargetPosition(pos)
        # self.SetTargetSpeed(vel)
        self.script_position_pd(pos, vel)

        position_and_velocitys = [(0, 20), (20, 40), (40, 40), (60, 20), (80, 20)]
        for position, velocity in position_and_velocitys:
            g_state = 0
            pos, vel = self.pre_position(position,velocity)
            # self.script_position_pd(position,velocity)
            self.SetTargetSpeed(int(vel))
            # Convert mm position to percentage (0-1000)
            position_percent = int((pos / 80.0) * 1000)
            self.SetTargetPosition(position_percent)
            # while g_state == 0:
            #     g_state = self.GetGripState()
            #     sleep(0.2)
            sleep(1e-2)
            g_position = self.GetCurrentPosition()
            print('g_position (percent)', g_position)
            print('g_position (mm)', g_position * 80 / 1000)
            g_velocity = self.GetCurrentTargetSpeed()
            print('g_velocity', g_velocity)
            print('gripper_state', g_state)
            print('===============================')
            sleep(delay_between_positions)

        for position, velocity in position_and_velocitys:
            g_state = 0
            # 使用 pre_position 方法
            self.script_position_pd(position, velocity, force)
            # 等待夹爪动作完成
            # while g_state == 0:
            #     g_state = self.GetGripState()
            #     sleep(0.43)
            sleep(2e-2)
            g_position = self.GetCurrentPosition()
            print('g_position (percent)', g_position)
            print('g_position (mm)', g_position * 80 / 1000)
            g_velocity = self.GetCurrentTargetSpeed()
            print('g_velocity', g_velocity)
            print('gripper_state', g_state)
            print('===============================')
            sleep(delay_between_positions)

        self.close()

if __name__ == '__main__':
    driver = DHBinaryDriver('192.168.58.18', 8887)
    driver.socket_gripper()


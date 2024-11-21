## 测试所有电机的工作电流
import datetime
import logging
import concurrent.futures
import os
import string
import sys
import threading
import time
from venv import logger

from pymodbus.exceptions import ConnectionException, ModbusIOException
from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient

# 设置日志级别为INFO，获取日志记录器实例
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# 检查log文件夹是否存在，如果不存在则创建
log_folder = "./log"
if not os.path.exists(log_folder):
    os.makedirs(log_folder)

# 创建一个文件处理器，用于将日志写入文件
file_handler = logging.FileHandler('./log/MotorCurrentTest_log.txt')
file_handler.setLevel(logging.INFO)

# 创建一个日志格式
log_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(log_format)

# 将文件处理器添加到日志记录器
logger.addHandler(file_handler)

stream_handler = logging.StreamHandler(stream=sys.stdout)
logger.addHandler(stream_handler)

class MotorCurrentTest:
    def __init__(self):
        self.node_id = 2
        self.port = 'COM4'
        self.FRAMER_TYPE = FramerType.RTU
        self.BAUDRATE = 115200
        self.client = None
        self.ROH_FINGER_POS_TARGET0 = 1135
        self.ROH_FINGER_CURRENT0 = 1105
        self.ROH_BEEP_PERIOD  = 1010
        self.max_average_times = 5
        self.initial_gesture = [0,0,0,0,0,0] #自然展开
        self.thumb_up_gesture = [0, 65535, 65535, 65535, 65535, 0] # 四指弯曲
        self.thumb_bend_gesture = [65535, 0, 0, 0, 0, 0] # 大拇值弯曲
        self.thumb_rotation_gesture = [0, 0, 0, 0, 0, 65535] # 大拇指旋转到对掌位

        self.gestures = self.create_gesture_dict()
        self.collectMotorCurrents = {
            'thumb':[0.0,0.0],
            'index':[0.0,0.0],
            'middle':[0.0,0.0],
            'third':[0.0,0.0],
            'little':[0.0,0.0],
            'thumb_root':[0.0,0.0]
        }
        
        self.min_motor_currents =[0.0,0.0,0.0,0.0,0.0,0.0]
        self.max_motor_currents =[0.0,0.0,0.0,0.0,0.0,0.0]
        
    def set_port(self,port):
        self.port = port
        
    def create_gesture_dict(self):
        gesture_dict = {
            '自然展开': self.initial_gesture,
            '四指弯曲': self.thumb_up_gesture,
            '大拇值弯曲': self.thumb_bend_gesture,
            '大拇指旋转到对掌位': self.thumb_rotation_gesture
        }
        return gesture_dict
    
    def read_from_register(self, address, count):
        """
        从指定的寄存器地址读取数据。

        最多尝试读取max_retries次，如果读取成功则返回读取结果。如果遇到连接超时或读取超时等错误，
        会进行相应处理（如重新连接或增加重试次数），其他异常也会被记录。

        :param address: 要读取的寄存器地址。
        :param count: 要读取的寄存器数量。
        :return: 如果成功读取则返回pymodbus的read_holding_registers响应对象，否则返回None。
        """
        max_retries = 3
        retry_count = 0
        response = None
        while retry_count < max_retries:
            try:
                response = self.client.read_holding_registers(address=address, count=count, slave=self.node_id)
                time.sleep(0.2)
                if not response.isError():
                    break
                else:
                    error_type = self.get_exception(response)
                    if "connection timeout" in error_type.lower():
                        self.client.connect()
                    elif "read timeout" in error_type.lower():
                        retry_count += 1
                        time.sleep(0.5)
                    else:
                        logger.error(f'[port = {self.port}]读寄存器失败: {error_type}\n')
            except ModbusIOException as e:
                logger.error(f'[port = {self.port}]Modbus输入输出异常: {e}')
                if "connection error" in str(e).lower():
                    self.client.connect()
            except AssertionError as e:
                logger.error(f'[port = {self.port}]断言错误: {e}')
            except Exception as e:
                logger.error(f'[port = {self.port}]其他异常: {e}')
        return response

    def write_to_regesister(self, address, value):
        """
        向指定的寄存器地址写入数据。

        最多尝试写入max_retries次，根据写入结果和可能出现的错误（如连接超时、写入超时等）进行相应处理，
        并返回写入是否成功的布尔值。

        :param address: 要写入的寄存器地址。
        :param value: 要写入的值。
        :return: 如果写入成功则返回True，否则返回False。
        """
        max_retries = 3
        retry_count = 0
        while retry_count < max_retries:
            try:
                response = self.client.write_registers(address, value, self.node_id)
                time.sleep(2)
                if not response.isError():
                    return True
                else:
                    error_type = self.get_exception(response)
                    if "connection timeout" in error_type.lower():
                        self.client.connect()
                    elif "write timeout" in error_type.lower():
                        retry_count += 1
                    else:
                        logger.error(f'[port = {self.port}]写寄存器失败: {error_type}\n')
                        return False
            except ModbusIOException as e:
                logger.error(f'[port = {self.port}]Modbus输入输出异常: {e}')
                if "connection error" in str(e).lower():
                    self.client.connect()
                else:
                    return False
            except AssertionError as e:
                logger.error(f'[port = {self.port}]断言错误: {e}')
                return False
            except Exception as e:
                logger.error(f'[port = {self.port}]其他异常: {e}')
                return False
        return False   
    
    # def do_alarm(self):
    #     """
    #     启动蜂鸣器报警功能
    #     每隔30s蜂鸣器报警一次
    #     """
    #      # 检查是否已经有正在运行的报警线程
    #     if hasattr(self, '_alarm_thread') and self._alarm_thread and self._alarm_thread.is_alive():
    #         return

    #     def alarm_thread_function():
    #         i = 0
    #         while i in range(5000):
    #             self.write_to_regesister(address=self.ROH_BEEP_PERIOD, value=3000)
    #             time.sleep(30)
    #             i += 1

    #     # 创建并启动新的线程
    #     self._alarm_thread = threading.Thread(target=alarm_thread_function)
    #     self._alarm_thread.start()

        
    def checkCurrent(self, curs):
        """
        检查电机电流是否超过100mA。

        遍历输入的电流列表，如果有任何一个电流值大于100则返回False，否则返回True。

        :param curs: 一个包含电机电流值的列表。
        :return: 一个布尔值，表示电流是否都在正常范围内。
        """
        return all(c <= 100 for c in curs)
    
    def connect_device(self):
        """
        连接到Modbus设备。

        创建ModbusSerialClient实例并尝试连接到指定端口的设备，根据连接结果记录日志并返回连接是否成功的布尔值。

        :return: 一个布尔值，表示是否成功连接到设备。
        """
        connect_status = False
        try:
            self.client = ModbusSerialClient(port=self.port, framer=self.FRAMER_TYPE, baudrate=self.BAUDRATE)
            connect_status = self.client.connect()
            logger.info(f"[port = {self.port}]Successfully connected to Modbus device.")
        except ConnectionException as e:
            logger.error(f"[port = {self.port}]Error during setup: {e}")
        except Exception as e:
            logger.error(f"[port = {self.port}]Error during setup: {e}")
        return connect_status

    def disConnect_device(self):
        """
        断开与Modbus设备的连接。

        如果存在client实例则关闭连接并将client设置为None，同时记录日志，如果出现异常也会记录。
        """
        if self.client:
            try:
                self.client.close()
                self.client = None
                logger.info(f"[port = {self.port}]Connection to Modbus device closed.")
            except Exception as e:
                logger.error(f"[port = {self.port}]Error during teardown: {e}")

    def do_gesture(self, key,gesture):
        """
        执行特定的手势动作。

        实际是向特定寄存器（ROH_FINGER_POS_TARGET0）写入手势数据。

        :param gesture: 要执行的手势数据。
        :return: 调用write_to_regesister方法的结果，即写入是否成功的布尔值。
        """
        return self.write_to_regesister(address=self.ROH_FINGER_POS_TARGET0, value=self.initial_gesture) and self.write_to_regesister(address=self.ROH_FINGER_POS_TARGET0, value=gesture)
    
    def count_motor_curtent(self):
        """
        计算电机电流的平均值。

        多次（最多MAX_NUM次）读取指定地址（ROH_FINGER_CURRENT0）的电流数据，然后计算这些数据的平均值并返回。

        :param address: 要读取电流数据的寄存器地址。
        :return: 一个包含6个电机电流平均值的列表。
        """
        sum_currents = [0] * 6
        ave_currents = [0] * 6
        MAX_NUM = self.max_average_times
        while MAX_NUM > 0:
            currents = self.read_from_register(address=self.ROH_FINGER_CURRENT0, count=6)
            if currents is None or currents.isError():
                logger.error("currents: read_holding_registers has an error \n")
            else:
                time.sleep(0.5)
            currents_list = currents.registers if currents else []
            sum_currents = [sum_currents[j] + currents_list[j] for j in range(len(currents_list))]
            MAX_NUM -= 1
        ave_currents = [sum_currents[k] / self.max_average_times for k in range(len(currents_list))]
       
        return ave_currents
    
    def collect_min_and_max_currents(self,ges='',current=[]):
        if ges == '自然展开':
            self.min_motor_currents = current
        elif ges == '四指弯曲':
            self.max_motor_currents[1:4] = current[1:4]
        elif ges == '大拇值弯曲':
            self.max_motor_currents[0] = current[0]
        elif ges == '大拇指旋转到对掌位':
            self.max_motor_currents[5] = current[5]
        else:
            logger.error('错误的手势')
            
    def collect_motor_currents(self):
        for key in self.collectMotorCurrents:
            index = list(self.collectMotorCurrents.keys()).index(key)
            self.collectMotorCurrents[key][0] = self.min_motor_currents[index]
            self.collectMotorCurrents[key][1] = self.max_motor_currents[index]
        
        # logger.info(f'port = {self.port}\n')
        print(*[f"[port = {self.port}][{key}]电机电流-->  <start> {values[0]}ma, <end> {values[1]}ma\n" for key, values in self.collectMotorCurrents.items()], sep='\n')

def check_ports(ports_list):
    valid_ports = []
    status = True
    if ports_list is not None:
        for port in ports_list:
            if isinstance(port, str) and port.startswith('COM'):
                valid_ports.append(port)
            else:
                status = False
    else:
        status = False
    return status, valid_ports

expected = [100,100,100,100,100,100]
description = '各个手指在始末位置,记录各个电机的电流值'

def main(ports=None, max_cycle_num=1):
    """
    测试的主函数。

    创建 AgeTest 类的实例，设置端口号并连接设备，然后进行多次（最多 MAX_CYCLE_NUM 次）测试循环，
    在每次循环中获取电机电流并检查电流是否正常，根据结果设置 result 变量，最后断开设备连接并返回测试结果。

    :param port: 可选参数，默认为 COM4，要连接的设备端口号。
    :return: 一个字符串，表示测试结果（"通过"或其他未在代码中明确设置的结果）。
    """
    test_title = '电机电流测试'
    final_result = '通过'
    overall_result = []
    connected_status = False
    need_show_current = True
    
    status, valid_ports = check_ports(ports)
    if not (status and len(valid_ports)>=1):
        logger.error('测试结束，无可用端口')
        final_result = '不通过'
        return test_title, overall_result, final_result, need_show_current
    
    start_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    logger.info(f'---------------------------------------------开始测试电机电流<开始时间：{start_time}>----------------------------------------------\n')
    logger.info('测试目的：各个手指在始末位置，各个电机的电流表现')
    logger.info('标准：电流值范围 < 0~100mA >\n')
    try:
        logger.info(f"##########################测试开始######################\n")
        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = [executor.submit(run_tests_for_port, port, connected_status) for port in ports]
            for future in concurrent.futures.as_completed(futures):
                port_result, _ = future.result()
                overall_result.append(port_result)
                for gesture_result in port_result["gestures"]:
                    if gesture_result["result"]!= "通过":
                        final_result = '不通过'
                        break
            logger.info(f"#################测试结束，测试结果：{final_result}#############\n")

    except Exception as e:
        logging.error(f"Error: {e}")
    finally:
        pass
    end_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    logger.info(f'---------------------------------------------电机电流测试结束<结束时间：{end_time}>----------------------------------------------\n')
    # print_overall_result(overall_result)
    return test_title, overall_result, final_result, need_show_current

def print_overall_result(overall_result):
        port_data_dict = {}

        # 整理数据
        for item in overall_result:
            if item['port'] not in port_data_dict:
                port_data_dict[item['port']] = []
            for gesture in item['gestures']:
                port_data_dict[item['port']].append((gesture['timestamp'],gesture['description'],gesture['expected'],gesture['content'], gesture['result'], gesture['comment']))

        # 打印数据
        for port, data_list in port_data_dict.items():
            logger.info(f"Port: {port}")
            for timestamp, description, expected, content, result, comment in data_list:
                logger.info(f" timestamp:{timestamp} ,description:{description},expected:{expected},content: {content}, Result: {result},comment:{comment}")


def run_tests_for_port(port, connected_status):
    result = '通过'
    motorCurrentTest = MotorCurrentTest()
    motorCurrentTest.set_port(port)
    if not connected_status:
        motorCurrentTest.connect_device()
        connected_status = True
    port_result = {
        "port": port,
        "gestures": []
    }
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    try:
        for key, gesture in motorCurrentTest.gestures.items():
            if motorCurrentTest.do_gesture(key = key, gesture = gesture):
                motors_current = motorCurrentTest.count_motor_curtent()
                motorCurrentTest.collect_min_and_max_currents(ges=key,current=motors_current)
                logger.info(f'[port = {port}]执行    ---->  {key},电机电流为 -->{motors_current}')
                if  not motorCurrentTest.checkCurrent(motors_current):
                    result = '不通过'
    except Exception as current_error:
        logger.error(f"获取电机电流或检查电流时出现错误：{current_error}")
        result = '不通过'
    motorCurrentTest.collect_motor_currents()
    gesture_result = {
                        "timestamp":timestamp,
                        "description":description,
                        "expected":expected,
                        "content": motorCurrentTest.collectMotorCurrents,
                        "result": result,
                        "comment":'无'
                        }
    port_result["gestures"].append(gesture_result)
    motorCurrentTest.disConnect_device()
    return port_result, connected_status
            
if __name__ == "__main__":
    port = ['COM4']
    main(ports = port,max_cycle_num=1)
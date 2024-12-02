import datetime
import json
import logging
import concurrent.futures
import os
import sys
import time
from typing import List, Optional, Tuple
from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ConnectionException, ModbusIOException

# 设置日志级别为INFO，获取日志记录器实例
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# 检查log文件夹是否存在，如果不存在则创建
log_folder = "./log"
if not os.path.exists(log_folder):
    os.makedirs(log_folder)

# 创建一个文件处理器，用于将日志写入文件
file_handler = logging.FileHandler('./log/AgingTest_log.txt')
file_handler.setLevel(logging.INFO)

# 创建一个日志格式
log_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(log_format)

# 将文件处理器添加到日志记录器
logger.addHandler(file_handler)

stream_handler = logging.StreamHandler(stream=sys.stdout)
logger.addHandler(stream_handler)


class AgingTest:
    
    # ROH 灵巧手错误代码
    EC01_ILLEGAL_FUNCTION = 0X1  # 无效的功能码
    EC02_ILLEGAL_DATA_ADDRESS = 0X2  # 无效的数据地址
    EC03_ILLEGAL_DATA_VALUE = 0X3  # 无效的数据（协议层，非应用层）
    EC04_SERVER_DEVICE_FAILURE = 0X4  # 设备故障
    UNKNOWN_FAILURE = 0X5  # 未知错误

    roh_exception_list = {
        EC01_ILLEGAL_FUNCTION: '无效的功能码',
        EC02_ILLEGAL_DATA_ADDRESS: '无效的数据地址',
        EC03_ILLEGAL_DATA_VALUE: '无效的数据（协议层，非应用层）',
        EC04_SERVER_DEVICE_FAILURE: '设备故障',
        UNKNOWN_FAILURE: '未知错误'
    }
    # 寄存器 ROH_SUB_EXCEPTION 保存了具体的错误代码
    ERR_STATUS_INIT = 0X1  # 等待初始化或者正在初始化，不接受此读写操作
    ERR_STATUS_CALI = 0X2  # 等待校正，不接受此读写操作
    ERR_INVALID_DATA = 0X3  # 无效的寄存器值
    ERR_STATUS_STUCK = 0X4  # 电机堵转
    ERR_OP_FAILED = 0X5  # 操作失败
    ERR_SAVE_FAILED = 0X6  # 保存失败
    ROH_SUB_EXCEPTION         = (1006) # R

    roh_sub_exception_list = {
        ERR_STATUS_INIT: '等待初始化或者正在初始化，不接受此读写操作',
        ERR_STATUS_CALI: '等待校正，不接受此读写操作',
        ERR_INVALID_DATA: '无效的寄存器值',
        ERR_STATUS_STUCK: '电机堵转',
        ERR_OP_FAILED: '操作失败',
        ERR_SAVE_FAILED: '保存失败'
    }
    

    def get_exception(self, response):
        """
        根据传入的响应确定错误类型。

        参数：
        response：包含错误信息的响应对象。

        返回：
        错误类型的描述字符串。
        """
        strException = ''
        if response.exception_code > self.EC04_SERVER_DEVICE_FAILURE:
            strException = self.roh_exception_list.get(self.UNKNOWN_FAILURE)
        elif response.exception_code == self.EC04_SERVER_DEVICE_FAILURE:
            response2 = self.client.read_holding_registers(address=self.ROH_SUB_EXCEPTION,slave=2)
            strException = '设备故障，具体原因为'+self.roh_sub_exception_list.get(response2.registers[0])
        else:
            strException = self.roh_exception_list.get(response.exception_code)
        return strException
    
    def __init__(self):
        """
        初始化AgeTest类的实例。

        在这里设置与Modbus设备通信相关的参数，包括端口号、节点ID、帧类型、波特率等，
        同时定义了与手指位置和电机电流相关的寄存器地址、初始手势、握手势、最大循环次数等属性。
        """
        self.node_id = 2
        self.port = 'COM4'
        self.FRAMER_TYPE = FramerType.RTU
        self.BAUDRATE = 115200
        self.client = None
        self.ROH_FINGER_POS_TARGET0 = 1135
        self.ROH_FINGER_CURRENT0 = 1105
        self.ROH_FINGER_CURRENT_LIMIT0 = 1095
        self.ROH_BEEP_PERIOD  = 1010
        self.motor_currents = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.initial_gesture = [[0,65535, 65535, 65535, 65535, 62258],[0, 0, 0, 0, 0, 62258]]  # 自然展开手势
        # self.grasp_gesture = [16294, 28966, 33673, 29328, 23897, 65535]  # 握手势
        # self.grasp_gesture = [65535, 65535, 65535, 65535, 65535, 65535]  # 握手势16294
        self.grasp_gesture = [[0, 65535, 65535, 65535, 65535, 62258], [62258, 65535, 65535, 65535, 65535, 62258]]
        self.FINGER_POS_TARGET_MAX_LOSS = 32
        self.max_average_times = 5
        self.current_standard = 100
        self.aging_speed = 1# 动作间隔，最少0.4，否则手指会碰撞，值越小越快
        
    def read_from_register(self, address, count):
        """
        从指定的寄存器地址读取数据。
        :param address: 要读取的寄存器地址。
        :param count: 要读取的寄存器数量。
        :return: 如果成功读取则返回pymodbus的read_holding_registers响应对象，否则返回None。
        """
        try:
            response = self.client.read_holding_registers(address=address, count=count, slave=self.node_id)
            if response.isError():
                error_type = self.get_exception(response)
                logger.error(f'[port = {self.port}]读寄存器失败: {error_type}\n')
        except Exception as e:
            logger.error(f'[port = {self.port}]异常: {e}')
        return response
        
    def write_to_regesister(self, address, value):
        """
        向指定的寄存器地址写入数据。
        :param address: 要写入的寄存器地址。
        :param value: 要写入的值。
        :return: 如果写入成功则返回True，否则返回False。
        """
        try:
            response = self.client.write_registers(address, value, self.node_id)
            if not response.isError():
                    return True
            else:
                error_type = self.get_exception(response)
                logger.error(f'[port = {self.port}]写寄存器失败: {error_type}\n')
                return False
        except Exception as e:
                logger.error(f'[port = {self.port}]异常: {e}')
                return False
    
    def do_gesture(self, gesture):
        """
        执行特定的手势动作。
        :param gesture: 要执行的手势数据。
        :return: 调用write_to_regesister方法的结果，即写入是否成功的布尔值。
        """
        time.sleep(self.aging_speed) # 防止大拇指和食指打架，值需要大于0.4
        return self.write_to_regesister(address=self.ROH_FINGER_POS_TARGET0, value=gesture)

    def count_motor_curtent(self, address):
        """
        计算电机电流的平均值。
        :param address: 要读取电流数据的寄存器地址。
        :return: 一个包含6个电机电流平均值的列表。
        """
        sum_currents = [0] * 6
        ave_currents = [0] * 6
        for i in range(self.max_average_times):
            currents = self.read_from_register(address=address, count=6)
            if currents is None or currents.isError():
                logger.error(f"[port = {self.port}]currents: read_holding_registers has an error\n")
            else:
                time.sleep(0.1) #每隔100ms读取一次电流值，然后取平均值
            currents_list = currents.registers if currents else []
            sum_currents = [sum_currents[j] + currents_list[j] for j in range(len(currents_list))]
        currents = [sum_currents[k] / self.max_average_times for k in range(len(currents_list))]
        ave_currents = [round(num, 1) for num in currents]
        return ave_currents

    def checkCurrent(self, curs):
        """
        检查电机电流是否超过标准<100mA>
        :param curs: 一个包含电机电流值的列表。
        :return: 一个布尔值，表示电流是否都在正常范围内。
        """
        return all(c <= self.current_standard for c in curs)
    
    def set_max_current(self):
        try:
            value = [200,200,200,200,200,200]
            response = self.client.write_registers(self.ROH_FINGER_CURRENT_LIMIT0, value, self.node_id)
            if not response.isError():
                return True
            else:
                error_type = self.get_exception(response)
                logger.error(f'[port = {self.port}]写寄存器失败: {error_type}\n')
                return False
        except Exception as e:
                logger.error(f'[port = {self.port}]异常: {e}')
                return False

    def get_motor_currents(self):
        """
        获取电机电流。
        :return: 一个布尔值，表示获取电机电流的操作是否成功。
        """
        status = False
        if self.do_gesture(self.grasp_gesture[0]) and self.do_gesture(self.grasp_gesture[1]):
            self.motor_currents = self.count_motor_curtent(address=self.ROH_FINGER_CURRENT0)
            logger.info(f'[port = {self.port}]执行抓握手势，电机电流为 -->{self.motor_currents}\n')
        if self.do_gesture(self.initial_gesture[0]) and self.do_gesture(self.initial_gesture[1])and not self.judge_if_hand_broken(self.ROH_FINGER_POS_TARGET0,self.initial_gesture[1]):
            status = True
        return status

    def get_current(self):
        """
        返回当前存储的电机电流值（self.motor_currents）。

        :return: 一个包含电机电流值的列表。
        """
        return self.motor_currents

    def judge_if_hand_broken(self, address, gesture):
        """
        判断设备是否损坏。

        通过读取指定地址的寄存器数据，并与给定的手势数据对比，如果有任何一个寄存器值与手势值的差值超过FINGER_POS_TARGET_MAX_LOSS则认为设备损坏。

        :param address: 要读取数据的寄存器地址。
        :param gesture: 用于对比的手势数据。
        :return: 一个布尔值，表示设备是否损坏。
        """
        is_broken = False
        response = self.read_from_register(address=address, count=6)
        if response is not None and not response.isError():
            for i in range(len(response.registers)):
                if abs(response.registers[i] - gesture[i]) > self.FINGER_POS_TARGET_MAX_LOSS:
                    is_broken = True
        return is_broken

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
            logger.info(f"[port = {self.port}]Successfully connected to Modbus device.\n")
        except ConnectionException as e:
            logger.error(f"Error during setup[port = {self.port}]: {e}\n")
        except Exception as e:
            logger.error(f"Error during setup[port = {self.port}]: {e}\n")
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
                logger.info(f"[port = {self.port}]Connection to Modbus device closed.\n")
            except Exception as e:
                logger.error(f"[port = {self.port}]Error during teardown: {e}\n")

def read_from_json_file():
        """
        从json文件读取标志位，判断是否继续执行测试
        Returns:
            返回fasle则终止测试，true继续测试
        """
        try:
            with open('shared_data.json', 'r') as f:
                data = json.load(f)
                stop_test = data['stop_test']
                pause_test = data['pause_test']
                return stop_test,pause_test
        except FileNotFoundError:
            # logger.error("共享的json文件不存在")
            return False,False
        except json.JSONDecodeError:
            # logger.error("json文件数据格式错误")
            return False,False
        except Exception as e:
            # logger.error(f"读取JSON文件时出现其他未知错误: {e}")
            return False, False

test_title = '老化测试报告\n标准：各个手头无异常，手指不脱线，并记录各个电机的电流值 < 单位 mA >'
expected = [100, 100, 100, 100, 100, 100]
description = '重复抓握手势,记录各个电机的电流值'  # 用例描述

# 定义一个常量用于表示老化测试的时长单位转换（从小时转换为秒）
SECONDS_PER_HOUR = 3600

def main(ports: list = [], node_ids: list = [], aging_duration: float = 1.5) -> Tuple[str, List, str, bool]:
    """
    测试的主函数。
    :param ports: 端口列表
    :param node_ids: 设备id列表,与端口号一一对应
    :return: 测试标题,测试结果数据,测试结论,是否需要显示电机电流(false)
    """
    overall_result = []
    final_result = '通过'
    start_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    logger.info(f'---------------------------------------------开始老化测试<开始时间：{start_time}>----------------------------------------------\n')
    logger.info('测试目的：循环做抓握手势，进行压测')
    logger.info('标准：各个手头无异常，手指不脱线，并记录各个电机的电流值 < 单位 mA >\n')
    try:
        end_time = time.time() + aging_duration * SECONDS_PER_HOUR
        round_num = 0
        while time.time() < end_time:
            round_num += 1
            logger.info(f"##########################第 {round_num} 轮测试开始######################\n")
            result = '通过'
            stop_test,pause_test = read_from_json_file()
            if stop_test:
                logger.info('测试已停止')
                break
                
            if pause_test:
                logger.info('测试暂停')
                time.sleep(2)
                continue

            round_results = []
            with concurrent.futures.ThreadPoolExecutor() as executor:
                futures = [executor.submit(test_single_port, port, node_id) for port, node_id in zip(ports, node_ids)]
                for future in concurrent.futures.as_completed(futures):
                    port_result, _ = future.result()
                    round_results.append(port_result)
                    for gesture_result in port_result["gestures"]:
                        if gesture_result["result"]!= "通过":
                            result = '不通过'
                            final_result = '不通过'
                            break

            overall_result.extend(round_results)
            logger.info(f"#################第 {round_num} 轮测试结束，测试结果：{result}#############\n")
    except Exception as e:
        final_result = '不通过'
        logger.error(f"Error: {e}")
    # finally:
    #     logger.info("执行测试结束后的清理操作（如有）")
    end_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    logger.info(f'---------------------------------------------老化测试结束<结束时间：{end_time}>----------------------------------------------\n')

    return test_title, overall_result, final_result, False

def test_single_port(port, node_id):
    """
    针对单个端口进行测试，返回该端口测试结果的字典，包含端口号、是否通过及具体手势测试结果等信息。
    """
    aging_test = AgingTest()
    aging_test.port = port
    aging_test.node_id = node_id
    
    connected_status = aging_test.connect_device()
    
    port_result = {
        'port': port,
        'gestures': []
    }
    
    if connected_status:
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        try:
            aging_test.set_max_current()
            if aging_test.get_motor_currents():
                current = aging_test.get_current()
                gesture_result = build_gesture_result(timestamp =timestamp,content=current,result='通过',comment='无')
            else:
                gesture_result = build_gesture_result(timestamp =timestamp,content='',result='不通过',comment='手指出现异常')
            port_result['gestures'].append(gesture_result)
        except Exception as e:
            error_gesture_result = build_gesture_result(timestamp =timestamp,content='',result='不通过',comment=f'出现错误：{e}')
            port_result['gestures'].append(error_gesture_result)
        finally:
            aging_test.disConnect_device()
    return port_result,connected_status

def build_gesture_result(timestamp,content,result,comment):
    return {
            "timestamp": timestamp,
            "description": description,
            "expected": expected,
            "content": content,
            "result": result,
            "comment": comment
        }

if __name__ == "__main__":
    ports = ['COM4']
    node_ids = [2]
    aging_duration = 0.01
    main(ports = ports, node_ids = node_ids, aging_duration = aging_duration)
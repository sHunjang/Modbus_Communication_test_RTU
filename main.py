# ============================================================================
# Modbus RTU 슬레이브 시뮬레이터 (PySide6 기반)
# ============================================================================
# 주요 수정 사항:
# 1. 32비트 Big-Endian 데이터 처리 구현
# 2. 3상4선 딕셔너리 방식 (주소 기반 매핑)
# 3. 백의 자리만 랜덤값 (0~9 × 100)
# 4. 전체 유효전력량 일의 자리 +1 증가
# ============================================================================

import datetime
import random
import sys
import time
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, Qt, QThread, Signal, Slot
from PySide6.QtWidgets import (QApplication, QMainWindow, QPushButton, QTableWidgetItem)
from main_ui import Ui_MainWindow

# ============================================================================
# 센서 타입 상수 정의
# ============================================================================
CONST_1P2W = 0x0a
CONST_3P3W = 0x09
CONST_3P4W = 0x08
CONST_OIL = 0x05
CONST_WATER = 0x04
CONST_CO2 = 0x01
CONST_TEMP = 0x03
CONST_SOLAR = 0x02
CONST_DC = 0x0b

name_list = {
    CONST_1P2W: "단상",
    CONST_3P3W: "3상3선",
    CONST_3P4W: "3상4선",
    CONST_OIL: "유량",
    CONST_WATER: "수도",
    CONST_CO2: "CO2",
    CONST_TEMP: "온습도",
    CONST_SOLAR: "일사량",
    CONST_DC: "DC모터",
}

values_list = {
    CONST_1P2W: [0x08A4, 0x04D2, 0x3039, 0x0000, 0x3039],
    CONST_3P3W: [0x0ED9, 0x0EE3, 0x0EEC, 0x04D2, 0x0929, 0x000A, 0x3039, 0x5BA0, 0x03E8, 0x0000, 0x3039, 0x0000, 0x5BA0, 0x0098, 0x967F],
    CONST_3P4W: [0x0ED9, 0x0EE3, 0x0EEC, 0x04D2, 0x0929, 0x000A, 0x04D2, 0x3039, 0x5BA0, 0x03E8, 0x0000, 0x3039, 0x0000, 0x5BA0, 0x0098, 0x967F],
    CONST_OIL: [0x00BC, 0x614E, 0x00BC, 0x614E],
    CONST_WATER: [0x000F, 0x423F],
    CONST_CO2: [0x07D0],
    CONST_TEMP: [0xFF85, 0x03E7],
    CONST_SOLAR: [0x4E20],
    CONST_DC: [0x0001],
}

CONST_LIST = [CONST_1P2W, CONST_3P3W, CONST_3P4W, CONST_OIL, CONST_WATER, CONST_CO2, CONST_TEMP, CONST_SOLAR, CONST_DC]

# ============================================================================
# Modbus 슬레이브 레지스터 저장소
# ============================================================================
store = {
    # 단상 (리스트 방식)
    (CONST_1P2W*16)+0: [2200, 1000, 20000, 0x01, 0x86a0],
    (CONST_1P2W*16)+1: [2210, 1100, 21000, 0x03, 0x0d40],
    (CONST_1P2W*16)+2: [2220, 1200, 22000, 0x04, 0x93e0],
    (CONST_1P2W*16)+3: [2230, 1300, 23000, 0x06, 0x1a80],
    (CONST_1P2W*16)+4: [2240, 1400, 24000, 0x07, 0xa120],
    (CONST_1P2W*16)+5: [2250, 1500, 25000, 0x09, 0x27c0],
    (CONST_1P2W*16)+6: [2260, 1600, 26000, 0x0a, 0xae60],
    (CONST_1P2W*16)+7: [2270, 1700, 27000, 0x0c, 0x3500],
    (CONST_1P2W*16)+8: [2280, 1800, 28000, 0x0d, 0xbba0],
    (CONST_1P2W*16)+9: [2290, 1900, 29000, 0x0f, 0x4240],

    # 3상3선 (리스트 방식)
    (CONST_3P3W*16)+0: [3800, 3800, 3800, 1000, 1000, 1000, 20000, 20000, 20000, 0x01, 0x86a0, 0x01, 0x86a0, 0x01, 0x86a0],
    (CONST_3P3W*16)+1: [3810, 3810, 3810, 1100, 1100, 1100, 21000, 21000, 21000, 0x03, 0x0d40, 0x03, 0x0d40, 0x03, 0x0d40],
    (CONST_3P3W*16)+2: [3820, 3820, 3820, 1200, 1200, 1200, 22000, 22000, 22000, 0x04, 0x93e0, 0x04, 0x93e0, 0x04, 0x93e0],
    (CONST_3P3W*16)+3: [3830, 3830, 3830, 1300, 1300, 1300, 23000, 23000, 23000, 0x06, 0x1a80, 0x06, 0x1a80, 0x06, 0x1a80],
    (CONST_3P3W*16)+4: [3840, 3840, 3840, 1400, 1400, 1400, 24000, 24000, 24000, 0x07, 0xa120, 0x07, 0xa120, 0x07, 0xa120],
    (CONST_3P3W*16)+5: [3850, 3850, 3850, 1500, 1500, 1500, 25000, 25000, 25000, 0x09, 0x27c0, 0x09, 0x27c0, 0x09, 0x27c0],
    (CONST_3P3W*16)+6: [3860, 3860, 3860, 1600, 1600, 1600, 26000, 26000, 26000, 0x0a, 0xae60, 0x0a, 0xae60, 0x0a, 0xae60],
    (CONST_3P3W*16)+7: [3870, 3870, 3870, 1700, 1700, 1700, 27000, 27000, 27000, 0x0c, 0x3500, 0x0c, 0x3500, 0x0c, 0x3500],
    (CONST_3P3W*16)+8: [3880, 3880, 3880, 1800, 1800, 1800, 28000, 28000, 28000, 0x0d, 0xbba0, 0x0d, 0xbba0, 0x0d, 0xbba0],
    (CONST_3P3W*16)+9: [3890, 3890, 3890, 1900, 1900, 1900, 29000, 29000, 29000, 0x0f, 0x4240, 0x0f, 0x4240, 0x0f, 0x4240],
    (CONST_3P3W*16)+10: [3890, 3890, 3890, 1900, 1900, 1900, 29000, 29000, 29000, 0x0f, 0x4240, 0x0f, 0x4240, 0x0f, 0x4240],
    (CONST_3P3W*16)+11: [3890, 3890, 3890, 1900, 1900, 1900, 29000, 29000, 29000, 0x0f, 0x4240, 0x0f, 0x4240, 0x0f, 0x4240],
    (CONST_3P3W*16)+12: [3890, 3890, 3890, 1900, 1900, 1900, 29000, 29000, 29000, 0x0f, 0x4240, 0x0f, 0x4240, 0x0f, 0x4240],
    (CONST_3P3W*16)+13: [3890, 3890, 3890, 1900, 1900, 1900, 29000, 29000, 29000, 0x0f, 0x4240, 0x0f, 0x4240, 0x0f, 0x4240],
    (CONST_3P3W*16)+14: [3890, 3890, 3890, 1900, 1900, 1900, 29000, 29000, 29000, 0x0f, 0x4240, 0x0f, 0x4240, 0x0f, 0x4240],
    (CONST_3P3W*16)+15: [3890, 3890, 3890, 1900, 1900, 1900, 29000, 29000, 29000, 0x0f, 0x4240, 0x0f, 0x4240, 0x0f, 0x4240],

    # 3상4선 (딕셔너리 방식 - TAC4300CT 주소 매핑)
    (CONST_3P4W*16)+0: {
        0x0024: 38000, 0x0026: 38001, 0x0028: 38002,  # 전압 L1/L2/L3
        0x0006: 10000, 0x0008: 10001, 0x000A: 10002,  # 전류 L1/L2/L3
        0x000C: 20000, 0x000E: 20001, 0x0010: 20002,  # 유효전력 L1/L2/L3
        0x0420: 30000, 0x0422: 30001, 0x0424: 30002,  # 전력량 L1/L2/L3
        0x0404: 654321,  # 전체 유효전력량
    },
    (CONST_3P4W*16)+1: {
        0x0024: 38010, 0x0026: 38011, 0x0028: 38012,
        0x0006: 10010, 0x0008: 10011, 0x000A: 10012,
        0x000C: 20010, 0x000E: 20011, 0x0010: 20012,
        0x0420: 30010, 0x0422: 30011, 0x0424: 30012,
        0x0404: 654321,
    },
    (CONST_3P4W*16)+2: {
        0x0024: 38020, 0x0026: 38021, 0x0028: 38022,
        0x0006: 10020, 0x0008: 10021, 0x000A: 10022,
        0x000C: 20020, 0x000E: 20021, 0x0010: 20022,
        0x0420: 30020, 0x0422: 30021, 0x0424: 30022,
        0x0404: 654321,
    },
    (CONST_3P4W*16)+3: {
        0x0024: 38030, 0x0026: 38031, 0x0028: 38032,
        0x0006: 10030, 0x0008: 10031, 0x000A: 10032,
        0x000C: 20030, 0x000E: 20031, 0x0010: 20032,
        0x0420: 30030, 0x0422: 30031, 0x0424: 30032,
        0x0404: 654321,
    },
    (CONST_3P4W*16)+4: {
        0x0024: 38040, 0x0026: 38041, 0x0028: 38042,
        0x0006: 10040, 0x0008: 10041, 0x000A: 10042,
        0x000C: 20040, 0x000E: 20041, 0x0010: 20042,
        0x0420: 30040, 0x0422: 30041, 0x0424: 30042,
        0x0404: 654321,
    },
    (CONST_3P4W*16)+5: {
        0x0024: 38050, 0x0026: 38051, 0x0028: 38052,
        0x0006: 10050, 0x0008: 10051, 0x000A: 10052,
        0x000C: 20050, 0x000E: 20051, 0x0010: 20052,
        0x0420: 30050, 0x0422: 30051, 0x0424: 30052,
        0x0404: 654321,
    },
    (CONST_3P4W*16)+6: {
        0x0024: 38060, 0x0026: 38061, 0x0028: 38062,
        0x0006: 10060, 0x0008: 10061, 0x000A: 10062,
        0x000C: 20060, 0x000E: 20061, 0x0010: 20062,
        0x0420: 30060, 0x0422: 30061, 0x0424: 30062,
        0x0404: 654321,
    },
    (CONST_3P4W*16)+7: {
        0x0024: 38070, 0x0026: 38071, 0x0028: 38072,
        0x0006: 10070, 0x0008: 10071, 0x000A: 10072,
        0x000C: 20070, 0x000E: 20071, 0x0010: 20072,
        0x0420: 30070, 0x0422: 30071, 0x0424: 30072,
        0x0404: 654321,
    },
    (CONST_3P4W*16)+8: {
        0x0024: 38080, 0x0026: 38081, 0x0028: 38082,
        0x0006: 10080, 0x0008: 10081, 0x000A: 10082,
        0x000C: 20080, 0x000E: 20081, 0x0010: 20082,
        0x0420: 30080, 0x0422: 30081, 0x0424: 30082,
        0x0404: 654321,
    },
    (CONST_3P4W*16)+9: {
        0x0024: 38090, 0x0026: 38091, 0x0028: 38092,
        0x0006: 10090, 0x0008: 10091, 0x000A: 10092,
        0x000C: 20090, 0x000E: 20091, 0x0010: 20092,
        0x0420: 30090, 0x0422: 30091, 0x0424: 30092,
        0x0404: 654321,
    },

    # 나머지 센서들 (기존 방식)
    (CONST_OIL*16)+0: [188, 24911, 188, 24911],
    (CONST_OIL*16)+1: [189, 24912, 189, 24912],
    (CONST_OIL*16)+2: [190, 24913, 190, 24913],
    (CONST_OIL*16)+3: [191, 24914, 191, 24914],
    (CONST_OIL*16)+4: [192, 24915, 192, 24915],
    (CONST_OIL*16)+5: [193, 24916, 193, 24916],
    (CONST_OIL*16)+6: [194, 24917, 194, 24917],
    (CONST_OIL*16)+7: [195, 24918, 195, 24918],
    (CONST_OIL*16)+8: [196, 24919, 196, 24919],
    (CONST_OIL*16)+9: [197, 24920, 197, 24920],

    (CONST_WATER*16)+0: [0, 16960],
    (CONST_WATER*16)+1: [1, 16961],
    (CONST_WATER*16)+2: [2, 16962],
    (CONST_WATER*16)+3: [3, 16963],
    (CONST_WATER*16)+4: [4, 16964],
    (CONST_WATER*16)+5: [5, 16965],
    (CONST_WATER*16)+6: [6, 16966],
    (CONST_WATER*16)+7: [7, 16967],
    (CONST_WATER*16)+8: [8, 16968],
    (CONST_WATER*16)+9: [9, 16969],

    (CONST_CO2*16)+0: [2001],
    (CONST_CO2*16)+1: [2002],
    (CONST_CO2*16)+2: [2003],
    (CONST_CO2*16)+3: [2004],
    (CONST_CO2*16)+4: [2005],
    (CONST_CO2*16)+5: [2006],
    (CONST_CO2*16)+6: [2007],
    (CONST_CO2*16)+7: [2008],
    (CONST_CO2*16)+8: [2009],
    (CONST_CO2*16)+9: [2010],

    (CONST_TEMP*16)+0: [65414, 999],
    (CONST_TEMP*16)+1: [65415, 998],
    (CONST_TEMP*16)+2: [65416, 997],
    (CONST_TEMP*16)+3: [65417, 996],
    (CONST_TEMP*16)+4: [65418, 995],
    (CONST_TEMP*16)+5: [65419, 994],
    (CONST_TEMP*16)+6: [65420, 993],
    (CONST_TEMP*16)+7: [65421, 992],
    (CONST_TEMP*16)+8: [65422, 991],
    (CONST_TEMP*16)+9: [65423, 990],

    (CONST_SOLAR*16)+0: [20001],
    (CONST_SOLAR*16)+1: [20002],
    (CONST_SOLAR*16)+2: [20003],
    (CONST_SOLAR*16)+3: [20004],
    (CONST_SOLAR*16)+4: [20005],
    (CONST_SOLAR*16)+5: [20006],
    (CONST_SOLAR*16)+6: [20007],
    (CONST_SOLAR*16)+7: [20008],
    (CONST_SOLAR*16)+8: [20009],
    (CONST_SOLAR*16)+9: [20010],

    (CONST_DC*16)+0: [0x00],
    (CONST_DC*16)+1: [0x01],
    (CONST_DC*16)+2: [0x81],
    (CONST_DC*16)+3: [0x00],
    (CONST_DC*16)+4: [0x01],
    (CONST_DC*16)+5: [0x81],
    (CONST_DC*16)+6: [0x00],
    (CONST_DC*16)+7: [0x01],
    (CONST_DC*16)+8: [0x81],
    (CONST_DC*16)+9: [0x00],
}


# ============================================================================
# CRC16 Modbus 계산 함수
# ============================================================================
def crc16_modbus(data: bytes) -> int:
    """Modbus RTU CRC16 계산"""
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


# ============================================================================
# 직렬 통신 수신 스레드
# ============================================================================
class com_thread(QThread):
    """QThread를 상속받아 별도 스레드에서 COM 포트 수신"""
    received_msg = Signal(bytes)
    log = Signal(str)

    def __init__(self):
        super().__init__()
        self.ser = serial.Serial()
        self.msg = bytes()

    def run(self):
        """무한 루프에서 8바이트씩 수신"""
        while True:
            try:
                self.msg = self.ser.read(8)
                if len(self.msg) < 1:
                    continue
                self.received_msg.emit(self.msg)
            except Exception as e:
                self.msg = []


# ============================================================================
# 직렬 통신 핸들러 클래스
# ============================================================================
class serial_handler(QObject):
    """QObject 기반 직렬 통신 관리"""
    log = Signal(str)

    def __init__(self):
        super().__init__()
        self.ser = None

    def get_port_list(self):
        """시스템에서 사용 가능한 COM 포트 목록 반환"""
        ports = serial.tools.list_ports.comports()
        available_ports = []
        for p in ports:
            available_ports.append(p.device)
        available_ports.sort()
        return available_ports

    def open(self, com_name, baud=9600):
        """COM 포트 열기"""
        try:
            self.ser = serial.Serial(com_name, baud, timeout=1)
            log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + ' COM PORT OPEN'
            self.log.emit(log_msg)
            return self.ser
        except Exception as e:
            log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + ' COM PORT OPEN FAIL'
            self.log.emit(log_msg)
            raise e

    def close(self):
        """COM 포트 닫기"""
        try:
            self.ser.close()
            log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + ' COM PORT CLOSE'
            self.log.emit(log_msg)
        except Exception as e:
            log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + ' COM PORT CLOSE FAIL'
            self.log.emit(log_msg)
            raise e


# ============================================================================
# 메인 윈도우 클래스
# ============================================================================
class MainWindow(QMainWindow, Ui_MainWindow):
    """Modbus RTU 슬레이브 시뮬레이터 메인 윈도우"""

    def __init__(self):
        """윈도우 초기화"""
        super().__init__()
        self.setupUi(self)

        # 전체 유효전력량 카운터 초기화 (일의 자리 증가용)
        self.total_power_counter = 0

        # 통신 객체 초기화
        self.comm_handler = serial_handler()
        self.worker = com_thread()

        # 신호-슬롯 연결
        self.worker.received_msg.connect(self.received_msg_slot)
        self.comm_handler.log.connect(self.log_slot)
        self.worker.log.connect(self.log_slot)

        # COM 포트 콤보박스 설정
        self.com_combo.addItems(self.comm_handler.get_port_list())
        default_baudrate_index = self.baudrate_combo.findText("9600")
        if default_baudrate_index >= 0:
            self.baudrate_combo.setCurrentIndex(default_baudrate_index)
        else:
            self.baudrate_combo.setCurrentIndex(0)

        # 버튼 이벤트 연결
        self.com_open_btn.clicked.connect(self.com_open_slot)
        self.com_close_btn.clicked.connect(self.com_close_slot)
        self.log_clear_btn.clicked.connect(self.log_clear_slot)

        # 센서별 그룹박스 매핑
        self.group_list = {
            CONST_1P2W: [self.tag1p2w_0_group, self.tag1p2w_1_group, self.tag1p2w_2_group,
                         self.tag1p2w_3_group, self.tag1p2w_4_group, self.tag1p2w_5_group,
                         self.tag1p2w_6_group, self.tag1p2w_7_group, self.tag1p2w_8_group,
                         self.tag1p2w_9_group],
            CONST_3P3W: [self.tag3p3w_0_group, self.tag3p3w_1_group, self.tag3p3w_2_group,
                         self.tag3p3w_3_group, self.tag3p3w_4_group, self.tag3p3w_5_group,
                         self.tag3p3w_6_group, self.tag3p3w_7_group, self.tag3p3w_8_group,
                         self.tag3p3w_9_group, self.tag3p3w_9_group, self.tag3p3w_9_group,
                         self.tag3p3w_9_group, self.tag3p3w_9_group, self.tag3p3w_9_group,
                         self.tag3p3w_9_group],
            CONST_3P4W: [self.tag3p4w_0_group, self.tag3p4w_1_group, self.tag3p4w_2_group,
                         self.tag3p4w_3_group, self.tag3p4w_4_group, self.tag3p4w_5_group,
                         self.tag3p4w_6_group, self.tag3p4w_7_group, self.tag3p4w_8_group,
                         self.tag3p4w_9_group],
            CONST_OIL: [self.oil_0_group, self.oil_1_group, self.oil_2_group, self.oil_3_group,
                        self.oil_4_group, self.oil_5_group, self.oil_6_group, self.oil_7_group,
                        self.oil_8_group, self.oil_9_group],
            CONST_WATER: [self.water_0_group, self.water_1_group, self.water_2_group,
                          self.water_3_group, self.water_4_group, self.water_5_group,
                          self.water_6_group, self.water_7_group, self.water_8_group,
                          self.water_9_group],
            CONST_CO2: [self.co2_0_group, self.co2_1_group, self.co2_2_group, self.co2_3_group,
                        self.co2_4_group, self.co2_5_group, self.co2_6_group, self.co2_7_group,
                        self.co2_8_group, self.co2_9_group],
            CONST_TEMP: [self.co2_0_group_2, self.co2_1_group_2, self.co2_2_group_2,
                         self.co2_3_group_2, self.co2_4_group_2, self.co2_5_group_2,
                         self.co2_6_group_2, self.co2_7_group_2, self.co2_8_group_2,
                         self.co2_9_group_2],
            CONST_SOLAR: [self.co2_0_group_3, self.co2_1_group_3, self.co2_2_group_3,
                          self.co2_3_group_3, self.co2_4_group_3, self.co2_5_group_3,
                          self.co2_6_group_3, self.co2_7_group_3, self.co2_8_group_3,
                          self.co2_9_group_3],
            CONST_DC: [self.co2_0_group_4, self.co2_1_group_4, self.co2_2_group_4,
                       self.co2_3_group_4, self.co2_4_group_4, self.co2_5_group_4,
                       self.co2_6_group_4, self.co2_7_group_4, self.co2_8_group_4,
                       self.co2_9_group_4],
        }

        # 센서별 라디오버튼 매핑
        self.radio_list = {
            CONST_1P2W: [
                [self.tag1p2w_0_normal_radio, self.tag1p2w_0_crc_radio, self.tag1p2w_0_timing_radio],
                [self.tag1p2w_1_normal_radio, self.tag1p2w_1_crc_radio, self.tag1p2w_1_timing_radio],
                [self.tag1p2w_2_normal_radio, self.tag1p2w_2_crc_radio, self.tag1p2w_2_timing_radio],
                [self.tag1p2w_3_normal_radio, self.tag1p2w_3_crc_radio, self.tag1p2w_3_timing_radio],
                [self.tag1p2w_4_normal_radio, self.tag1p2w_4_crc_radio, self.tag1p2w_4_timing_radio],
                [self.tag1p2w_5_normal_radio, self.tag1p2w_5_crc_radio, self.tag1p2w_5_timing_radio],
                [self.tag1p2w_6_normal_radio, self.tag1p2w_6_crc_radio, self.tag1p2w_6_timing_radio],
                [self.tag1p2w_7_normal_radio, self.tag1p2w_7_crc_radio, self.tag1p2w_7_timing_radio],
                [self.tag1p2w_8_normal_radio, self.tag1p2w_8_crc_radio, self.tag1p2w_8_timing_radio],
                [self.tag1p2w_9_normal_radio, self.tag1p2w_9_crc_radio, self.tag1p2w_9_timing_radio],
            ],
            CONST_3P3W: [
                [self.tag3p3w_0_normal_radio, self.tag3p3w_0_crc_radio, self.tag3p3w_0_timing_radio],
                [self.tag3p3w_1_normal_radio, self.tag3p3w_1_crc_radio, self.tag3p3w_1_timing_radio],
                [self.tag3p3w_2_normal_radio, self.tag3p3w_2_crc_radio, self.tag3p3w_2_timing_radio],
                [self.tag3p3w_3_normal_radio, self.tag3p3w_3_crc_radio, self.tag3p3w_3_timing_radio],
                [self.tag3p3w_4_normal_radio, self.tag3p3w_4_crc_radio, self.tag3p3w_4_timing_radio],
                [self.tag3p3w_5_normal_radio, self.tag3p3w_5_crc_radio, self.tag3p3w_5_timing_radio],
                [self.tag3p3w_6_normal_radio, self.tag3p3w_6_crc_radio, self.tag3p3w_6_timing_radio],
                [self.tag3p3w_7_normal_radio, self.tag3p3w_7_crc_radio, self.tag3p3w_7_timing_radio],
                [self.tag3p3w_8_normal_radio, self.tag3p3w_8_crc_radio, self.tag3p3w_8_timing_radio],
                [self.tag3p3w_9_normal_radio, self.tag3p3w_9_crc_radio, self.tag3p3w_9_timing_radio],
                [self.tag3p3w_9_normal_radio, self.tag3p3w_9_crc_radio, self.tag3p3w_9_timing_radio],
                [self.tag3p3w_9_normal_radio, self.tag3p3w_9_crc_radio, self.tag3p3w_9_timing_radio],
                [self.tag3p3w_9_normal_radio, self.tag3p3w_9_crc_radio, self.tag3p3w_9_timing_radio],
                [self.tag3p3w_9_normal_radio, self.tag3p3w_9_crc_radio, self.tag3p3w_9_timing_radio],
                [self.tag3p3w_9_normal_radio, self.tag3p3w_9_crc_radio, self.tag3p3w_9_timing_radio],
                [self.tag3p3w_9_normal_radio, self.tag3p3w_9_crc_radio, self.tag3p3w_9_timing_radio],
            ],
            CONST_3P4W: [
                [self.tag3p4w_0_normal_radio, self.tag3p4w_0_crc_radio, self.tag3p4w_0_timing_radio],
                [self.tag3p4w_1_normal_radio, self.tag3p4w_1_crc_radio, self.tag3p4w_1_timing_radio],
                [self.tag3p4w_2_normal_radio, self.tag3p4w_2_crc_radio, self.tag3p4w_2_timing_radio],
                [self.tag3p4w_3_normal_radio, self.tag3p4w_3_crc_radio, self.tag3p4w_3_timing_radio],
                [self.tag3p4w_4_normal_radio, self.tag3p4w_4_crc_radio, self.tag3p4w_4_timing_radio],
                [self.tag3p4w_5_normal_radio, self.tag3p4w_5_crc_radio, self.tag3p4w_5_timing_radio],
                [self.tag3p4w_6_normal_radio, self.tag3p4w_6_crc_radio, self.tag3p4w_6_timing_radio],
                [self.tag3p4w_7_normal_radio, self.tag3p4w_7_crc_radio, self.tag3p4w_7_timing_radio],
                [self.tag3p4w_8_normal_radio, self.tag3p4w_8_crc_radio, self.tag3p4w_8_timing_radio],
                [self.tag3p4w_9_normal_radio, self.tag3p4w_9_crc_radio, self.tag3p4w_9_timing_radio],
            ],
            CONST_OIL: [
                [self.oil_0_normal_radio, self.oil_0_crc_radio, self.oil_0_timing_radio],
                [self.oil_1_normal_radio, self.oil_1_crc_radio, self.oil_1_timing_radio],
                [self.oil_2_normal_radio, self.oil_2_crc_radio, self.oil_2_timing_radio],
                [self.oil_3_normal_radio, self.oil_3_crc_radio, self.oil_3_timing_radio],
                [self.oil_4_normal_radio, self.oil_4_crc_radio, self.oil_4_timing_radio],
                [self.oil_5_normal_radio, self.oil_5_crc_radio, self.oil_5_timing_radio],
                [self.oil_6_normal_radio, self.oil_6_crc_radio, self.oil_6_timing_radio],
                [self.oil_7_normal_radio, self.oil_7_crc_radio, self.oil_7_timing_radio],
                [self.oil_8_normal_radio, self.oil_8_crc_radio, self.oil_8_timing_radio],
                [self.oil_9_normal_radio, self.oil_9_crc_radio, self.oil_9_timing_radio],
            ],
            CONST_WATER: [
                [self.water_0_normal_radio, self.water_0_crc_radio, self.water_0_timing_radio],
                [self.water_1_normal_radio, self.water_1_crc_radio, self.water_1_timing_radio],
                [self.water_2_normal_radio, self.water_2_crc_radio, self.water_2_timing_radio],
                [self.water_3_normal_radio, self.water_3_crc_radio, self.water_3_timing_radio],
                [self.water_4_normal_radio, self.water_4_crc_radio, self.water_4_timing_radio],
                [self.water_5_normal_radio, self.water_5_crc_radio, self.water_5_timing_radio],
                [self.water_6_normal_radio, self.water_6_crc_radio, self.water_6_timing_radio],
                [self.water_7_normal_radio, self.water_7_crc_radio, self.water_7_timing_radio],
                [self.water_8_normal_radio, self.water_8_crc_radio, self.water_8_timing_radio],
                [self.water_9_normal_radio, self.water_9_crc_radio, self.water_9_timing_radio],
            ],
            CONST_CO2: [
                [self.co2_0_normal_radio, self.co2_0_crc_radio, self.co2_0_timing_radio],
                [self.co2_1_normal_radio, self.co2_1_crc_radio, self.co2_1_timing_radio],
                [self.co2_2_normal_radio, self.co2_2_crc_radio, self.co2_2_timing_radio],
                [self.co2_3_normal_radio, self.co2_3_crc_radio, self.co2_3_timing_radio],
                [self.co2_4_normal_radio, self.co2_4_crc_radio, self.co2_4_timing_radio],
                [self.co2_5_normal_radio, self.co2_5_crc_radio, self.co2_5_timing_radio],
                [self.co2_6_normal_radio, self.co2_6_crc_radio, self.co2_6_timing_radio],
                [self.co2_7_normal_radio, self.co2_7_crc_radio, self.co2_7_timing_radio],
                [self.co2_8_normal_radio, self.co2_8_crc_radio, self.co2_8_timing_radio],
                [self.co2_9_normal_radio, self.co2_9_crc_radio, self.co2_9_timing_radio],
            ],
            CONST_TEMP: [
                [self.co2_0_normal_radio_2, self.co2_0_crc_radio_2, self.co2_0_timing_radio_2],
                [self.co2_1_normal_radio_2, self.co2_1_crc_radio_2, self.co2_1_timing_radio_2],
                [self.co2_2_normal_radio_2, self.co2_2_crc_radio_2, self.co2_2_timing_radio_2],
                [self.co2_3_normal_radio_2, self.co2_3_crc_radio_2, self.co2_3_timing_radio_2],
                [self.co2_4_normal_radio_2, self.co2_4_crc_radio_2, self.co2_4_timing_radio_2],
                [self.co2_5_normal_radio_2, self.co2_5_crc_radio_2, self.co2_5_timing_radio_2],
                [self.co2_6_normal_radio_2, self.co2_6_crc_radio_2, self.co2_6_timing_radio_2],
                [self.co2_7_normal_radio_2, self.co2_7_crc_radio_2, self.co2_7_timing_radio_2],
                [self.co2_8_normal_radio_2, self.co2_8_crc_radio_2, self.co2_8_timing_radio_2],
                [self.co2_9_normal_radio_2, self.co2_9_crc_radio_2, self.co2_9_timing_radio_2],
            ],
            CONST_SOLAR: [
                [self.co2_0_normal_radio_3, self.co2_0_crc_radio_3, self.co2_0_timing_radio_3],
                [self.co2_1_normal_radio_3, self.co2_1_crc_radio_3, self.co2_1_timing_radio_3],
                [self.co2_2_normal_radio_3, self.co2_2_crc_radio_3, self.co2_2_timing_radio_3],
                [self.co2_3_normal_radio_3, self.co2_3_crc_radio_3, self.co2_3_timing_radio_3],
                [self.co2_4_normal_radio_3, self.co2_4_crc_radio_3, self.co2_4_timing_radio_3],
                [self.co2_5_normal_radio_3, self.co2_5_crc_radio_3, self.co2_5_timing_radio_3],
                [self.co2_6_normal_radio_3, self.co2_6_crc_radio_3, self.co2_6_timing_radio_3],
                [self.co2_7_normal_radio_3, self.co2_7_crc_radio_3, self.co2_7_timing_radio_3],
                [self.co2_8_normal_radio_3, self.co2_8_crc_radio_3, self.co2_8_timing_radio_3],
                [self.co2_9_normal_radio_3, self.co2_9_crc_radio_3, self.co2_9_timing_radio_3],
            ],
            CONST_DC: [
                [self.co2_0_normal_radio_4, self.co2_0_crc_radio_4, self.co2_0_timing_radio_4],
                [self.co2_1_normal_radio_4, self.co2_1_crc_radio_4, self.co2_1_timing_radio_4],
                [self.co2_2_normal_radio_4, self.co2_2_crc_radio_4, self.co2_2_timing_radio_4],
                [self.co2_3_normal_radio_4, self.co2_3_crc_radio_4, self.co2_3_timing_radio_4],
                [self.co2_4_normal_radio_4, self.co2_4_crc_radio_4, self.co2_4_timing_radio_4],
                [self.co2_5_normal_radio_4, self.co2_5_crc_radio_4, self.co2_5_timing_radio_4],
                [self.co2_6_normal_radio_4, self.co2_6_crc_radio_4, self.co2_6_timing_radio_4],
                [self.co2_7_normal_radio_4, self.co2_7_crc_radio_4, self.co2_7_timing_radio_4],
                [self.co2_8_normal_radio_4, self.co2_8_crc_radio_4, self.co2_8_timing_radio_4],
                [self.co2_9_normal_radio_4, self.co2_9_crc_radio_4, self.co2_9_timing_radio_4],
            ],
        }

        self.show()

    @Slot(bytes)
    def received_msg_slot(self, msg):
        """COM 포트로부터 수신한 메시지 처리"""
        try:
            data = msg[:-2]
            crc = int.from_bytes(msg[-2:], byteorder='little')
            if crc == crc16_modbus(data):
                log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + ' OK RX: ' + ' '.join([f'{i:02x}' for i in msg])
                self.log_slot(log_msg)
                self.update_data(data)
            else:
                log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + ' Fail RX: ' + ' '.join([f'{i:02x}' for i in msg])
                self.log_slot(log_msg)
        except Exception as e:
            print('received_msg_slot', type(e).__name__, e)

    def update_data(self, data):
        """
        Modbus 요청 분석 및 응답 생성
        
        3상4선 특수 처리:
        - 딕셔너리 기반 (주소로 접근)
        - 백의 자리만 랜덤 (0~9 × 100)
        - 전체 유효전력량: 매번 +1 증가
        """
        
        def get_random_value(d_type, idx):
            """센서 타입별 랜덤값 생성"""
            ret = 0
            
            if d_type == CONST_1P2W:
                if idx == 0:
                    ret = random.randrange(0, 10)
                elif idx == 1:
                    ret = random.randrange(0, 100)
                elif idx == 2:
                    ret = random.randrange(0, 1000)
                elif idx == 4:
                    ret = random.randrange(0, 10000)
            
            elif d_type == CONST_3P3W:
                if 0 <= idx <= 2:
                    ret = random.randrange(0, 10)
                elif 3 <= idx <= 5:
                    ret = random.randrange(0, 100)
                elif 6 <= idx <= 9:
                    ret = random.randrange(0, 1000)
            
            # ★ 3상4선: 백의 자리만 (0~9) × 100
            elif d_type == CONST_3P4W:
                ret = random.randrange(0, 10) * 100  # 0, 100, 200, ..., 900
            
            elif d_type == CONST_OIL:
                ret = random.randrange(0, 1000)
            elif d_type == CONST_WATER:
                ret = random.randrange(0, 100)
            elif d_type == CONST_CO2:
                ret = random.randrange(0, 50)
            elif d_type == CONST_TEMP:
                ret = random.randrange(0, 50)
            elif d_type == CONST_SOLAR:
                ret = random.randrange(0, 1000)
            elif d_type == CONST_DC:
                ret = random.randrange(0, 10)
            
            return ret

        try:
            # ===== 요청 메시지 파싱 =====
            device_id = data[0]
            function_code = data[1]
            addr = int.from_bytes(data[2:4], byteorder='big')
            num_reg = int.from_bytes(data[4:6], byteorder='big')

            if function_code == 0x03 or function_code == 0x04:
                device_type = device_id // 16
                device_addr = device_id % 16

                byte_count = num_reg * 2
                send_msg = [device_id, function_code, byte_count]

                store_data = store[device_id]
                
                # ★ 3상4선: 딕셔너리 (주소 기반)
                if isinstance(store_data, dict):
                    for i in range(num_reg):
                        requested_addr = addr + i
                        
                        # 주소로 직접 접근
                        if requested_addr in store_data:
                            base_value = store_data[requested_addr]
                        else:
                            base_value = 0
                        
                        # 데이터 변조
                        if requested_addr == 0x0404:
                            # 0x0404: 일의 자리 +1 증가
                            self.total_power_counter += 1
                            value = (base_value + self.total_power_counter) & 0xFFFF
                        else:
                            # 일반 주소: 백의 자리만 랜덤 추가
                            random_val = get_random_value(device_type, i)
                            value = (base_value + random_val) & 0xFFFF
                        
                        # 16비트 Big-Endian
                        send_msg.append((value >> 8) & 0xFF)
                        send_msg.append(value & 0xFF)
                
                # ★ 기타 센서: 리스트 (인덱스 기반)
                else:
                    for i in range(0, num_reg, 2):
                        store_idx = i
                        
                        if store_idx < len(store_data):
                            value_32bit = store_data[store_idx] + get_random_value(device_type, store_idx)
                        else:
                            value_32bit = 0
                        
                        # 32비트 Big-Endian 처리
                        high_word = (value_32bit >> 16) & 0xFFFF
                        low_word = value_32bit & 0xFFFF
                        
                        send_msg.append((high_word >> 8) & 0xFF)
                        send_msg.append(high_word & 0xFF)
                        send_msg.append((low_word >> 8) & 0xFF)
                        send_msg.append(low_word & 0xFF)

                # ===== CRC 계산 =====
                crc = crc16_modbus(send_msg)
                send_msg.append((crc & 0x00FF))
                send_msg.append((crc & 0xFF00) >> 8)

                # ===== 오류 주입 처리 =====
                if self.group_list[device_type][device_addr].isChecked():
                    if self.radio_list[device_type][device_addr][0].isChecked():
                        self.log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + ' OK TX: '
                    
                    elif self.radio_list[device_type][device_addr][1].isChecked():
                        self.log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + f'{name_list[device_type]} {device_addr}번 CRC ERROR TX: '
                        send_msg[-1] ^= 0x5c
                    
                    elif self.radio_list[device_type][device_addr][2].isChecked():
                        self.log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + f'{name_list[device_type]} {device_addr}번 TIMING ERROR TX: '
                        time.sleep(self.timing_delay_spin.value() / 1000)

                self.check_and_write(send_msg)

            else:
                device_type = device_id // 16
                device_addr = device_id % 16
                self.log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + f'{name_list[device_type]} {device_addr}번 NO CHECK TX'
                self.log_slot(self.log_msg)

        except KeyError:
            self.log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + ' ID out of range : ' + f'{device_id:02X}'
            self.log_slot(self.log_msg)

        except IndexError:
            device_type = device_id // 16
            device_addr = device_id % 16
            self.log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + f' {name_list[device_type]} {device_addr}번 Read Register out of range'
            self.log_slot(self.log_msg)

        except Exception as e:
            log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + " Error: " + type(e).__name__
            self.log_slot(log_msg)


    def check_and_write(self, send_msg):
        """응답 메시지를 COM 포트로 전송"""
        try:
            self.worker.ser.write(send_msg)
            self.log_msg += ' '.join([f'{i:02x}' for i in send_msg])
            self.log_slot(self.log_msg)
        except Exception as e:
            log_msg = datetime.datetime.now().strftime("[%H:%M:%S]") + "Error: " + str(e)
            self.log_slot(log_msg)

    def com_open_slot(self):
        """COM 포트 열기 및 수신 스레드 시작"""
        try:
            self.worker.ser = self.comm_handler.open(
                self.com_combo.currentText(),
                int(self.baudrate_combo.currentText())
            )
            self.com_open_btn.setEnabled(False)
            self.com_close_btn.setEnabled(True)
            self.worker.start()
        except Exception as e:
            print(e)

    def com_close_slot(self):
        """COM 포트 닫기"""
        try:
            self.comm_handler.close()
            self.com_open_btn.setEnabled(True)
            self.com_close_btn.setEnabled(False)
        except Exception as e:
            print(e)

    @Slot(str)
    def log_slot(self, log_msg):
        """로그 메시지를 UI 텍스트 에디터에 출력"""
        self.log_text_edit.appendPlainText(log_msg)

    @Slot()
    def log_clear_slot(self):
        """Log 텍스트 내용 삭제"""
        try:
            self.log_text_edit.clear()
            log_msg = datetime.datetime.now().strftime("[%H:%M:%S]")
            self.log_text_edit.appendPlainText(log_msg)
        except Exception as e:
            print(f" Log 삭제 실패: {e} ")


# ============================================================================
# 메인 실행
# ============================================================================
if __name__ == '__main__':
    app = QApplication(sys.argv)
    mywindow = MainWindow()
    sys.exit(app.exec())

import platform
from ctypes import *
import threading
import time
lib = cdll.LoadLibrary("./libusbcan.so")

USBCAN_I = c_uint32(3)   # USBCAN-I/I+ 3
USBCAN_II = c_uint32(4)  # USBCAN-II/II+ 4
MAX_CHANNELS = 1         # Maximum number of channels
g_thd_run = 1            # Thread running flag


class ZCAN_CAN_BOARD_INFO(Structure):
    _fields_ = [("hw_Version", c_ushort),
                ("fw_Version", c_ushort),
                ("dr_Version", c_ushort),
                ("in_Version", c_ushort),
                ("irq_Num", c_ushort),
                ("can_Num", c_ubyte),
                ("str_Serial_Num", c_ubyte*20),
                ("str_hw_Type", c_ubyte*40),
                ("Reserved", c_ubyte*4)]

    def __str__(self):
        return "Hardware Version:%s\nFirmware Version:%s\nDriver Version:%s\nInterface:%s\nInterrupt Number:%s\nCAN_number:%d" % (
            self.hw_Version,  self.fw_Version,  self.dr_Version,  self.in_Version,  self.irq_Num,  self.can_Num)

    def serial(self):
        serial = ''
        for c in self.str_Serial_Num:
            if c > 0:
                serial += chr(c)
            else:
                break
        return serial

    def hw_Type(self):
        hw_Type = ''
        for c in self.str_hw_Type:
            if c > 0:
                hw_Type += chr(c)
            else:
                break
        return hw_Type


class ZCAN_CAN_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_int),
                ("AccMask", c_int),
                ("Reserved", c_int),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)]


class ZCAN_CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint32),
                ("TimeStamp", c_uint32),
                ("TimeFlag", c_uint8),
                ("SendType", c_byte),
                ("RemoteFlag", c_byte),
                ("ExternFlag", c_byte),
                ("DataLen", c_byte),
                ("Data", c_ubyte*8),
                ("Reserved", c_ubyte*3)]


def GetDeviceInf(DeviceType, DeviceIndex):
    try:
        info = ZCAN_CAN_BOARD_INFO()
        ret = lib.VCI_ReadBoardInfo(DeviceType, DeviceIndex, byref(info))
        return info if ret == 1 else None
    except:
        print("Exception on readboardinfo")
        raise


def rx_thread(DEVCIE_TYPE, DevIdx, chn_idx):
    global g_thd_run
    while g_thd_run == 1:
        time.sleep(0.1)
        count = lib.VCI_GetReceiveNum(DEVCIE_TYPE, DevIdx, chn_idx) # Get the number of messages in the buffer
        if count > 0:
            print("Number of messages in the buffer: %d" % count)
            can = (ZCAN_CAN_OBJ * count)()
            rcount = lib.VCI_Receive(DEVCIE_TYPE, DevIdx, chn_idx, byref(can), count, 100) # Read messages
            for i in range(rcount):
                print("[%d] %d ID: 0x%x " %(can[i].TimeStamp, chn_idx, can[i].ID), end='')
                print("%s " %("Extended frame" if can[i].ExternFlag == 1 else "Standard frame"), end='')
                if can[i].RemoteFlag == 0:
                    print(" Data: ", end='')
                    for j in range(can[i].DataLen):
                        print("%02x "% can[i].Data[j], end='')
                else:
                    print(" Remote frame", end='')
                print("")


if __name__ == "__main__":
    threads = []
    # The hexadecimal number for the baud rate here can be calculated using the "zcanpro baud rate calculator"
    gBaud = 0x1c00          # Baud rate 0x1400-1M(75%), 0x1c00-500k(87.5%), 0x1c01-250k(87.5%), 0x1c03-125k(87.5%)
    DevType = USBCAN_I      # Device type number
    DevIdx = 0              # Device index number

    # Open device
    ret = lib.VCI_OpenDevice(DevType, DevIdx, 0)   # Device type, device index, reserved parameter
    if ret == 0:
        print("Open device fail")
        exit(0)
    else:
        print("Open device success")

    # # Get device information
    # info = GetDeviceInf(USBCAN_II, 0)
    # print("Device Information:\n%s" % (info))

    # Initialize and start channels
    for i in range(MAX_CHANNELS):
        init_config = ZCAN_CAN_INIT_CONFIG()
        init_config.AccCode = 0
        init_config.AccMask = 0xFFFFFFFF
        init_config.Reserved = 0
        init_config.Filter = 1
        init_config.Timing0 = gBaud & 0xff
        init_config.Timing1 = gBaud >> 8
        init_config.Mode = 2
        ret = lib.VCI_InitCAN(DevType, 0, i, byref(init_config))
        if ret == 0:
            print("InitCAN(%d) fail" % i)
        else:
            print("InitCAN(%d) success" % i)

        ret = lib.VCI_StartCAN(DevType, 0, i)
        if ret == 0:
            print("StartCAN(%d) fail" % i)
        else:
            print("StartCAN(%d) success" % i)

        thread = threading.Thread(target=rx_thread, args=(DevType, DevIdx, i,))
        threads.append(thread) # Independent receiving thread
        thread.start()

    # Test sending
    send_len = 10  # Number of frames to send
    msgs = (ZCAN_CAN_OBJ * send_len)()
    for i in range(send_len):
        msgs[i].ID = 0x100
        msgs[i].SendType = 2    # Sending method 0-Normal, 1-Single, 2-Self-send-self-receive
        msgs[i].RemoteFlag = 0  # 0-Data frame 1-Remote frame
        msgs[i].ExternFlag = 0  # 0-Standard frame 1-Extended frame
        msgs[i].DataLen = 8     # Data length 1~8
        for j in range(msgs[i].DataLen):
            msgs[i].Data[j] = j
    send_ret = lib.VCI_Transmit(DevType, 0, 0, byref(msgs), send_len)

    if send_len == send_ret:
        print("Transmit success, send count is: %d " % send_ret)
    else:
        print("Transmit fail, send count is: %d " % send_ret)

    # Block and wait
    input()
    g_thd_run = 0

    # Wait for all threads to complete
    for thread in threads:
        thread.join()

    # Reset channels
    for i in range(MAX_CHANNELS):
        ret = lib.VCI_ResetCAN(DevType, DevIdx, i)
        if ret == 0:
            print("ResetCAN(%d) fail" % i)
        else:
            print("ResetCAN(%d) success" % i)

    # Close device
    ret = lib.VCI_CloseDevice(DevType, DevIdx)
    if ret == 0:
        print("Close device fail")
    else:
        print("Close device success")
    del lib

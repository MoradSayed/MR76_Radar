"""
Python wrapper for ZLGCAN CAN interface library
Converted from controlcan.h

Created By: Morad Singer (Morad.S.Singer@gmail.com)
"""

from ctypes import * # type: ignore
from typing import Optional, Union, List, Tuple
import platform, time

# 接口卡类型定义 (Interface card type definitions)
VCI_PCI5121 = 1
VCI_PCI9810 = 2
VCI_USBCAN1 = 3
VCI_USBCAN2 = 4
VCI_PCI9820 = 5
VCI_CAN232  = 6
VCI_PCI5110 = 7
VCI_CANLITE = 8
VCI_ISA9620 = 9
VCI_ISA5420 = 10

# CAN错误码 (CAN error codes)
ERR_CAN_OVERFLOW = 0x0001  # CAN控制器内部FIFO溢出
ERR_CAN_ERRALARM = 0x0002  # CAN控制器错误报警
ERR_CAN_PASSIVE = 0x0004   # CAN控制器消极错误
ERR_CAN_LOSE = 0x0008      # CAN控制器仲裁丢失
ERR_CAN_BUSERR = 0x0010    # CAN控制器总线错误
ERR_CAN_REG_FULL = 0x0020  # CAN接收寄存器满
ERR_CAN_REC_OVER = 0x0040  # CAN接收寄存器溢出
ERR_CAN_ACTIVE = 0x0080    # CAN控制器主动错误

# 通用错误码 (General error codes)
ERR_DEVICEOPENED = 0x0100     # 设备已经打开
ERR_DEVICEOPEN = 0x0200       # 打开设备错误
ERR_DEVICENOTOPEN = 0x0400    # 设备没有打开
ERR_BUFFEROVERFLOW = 0x0800   # 缓冲区溢出
ERR_DEVICENOTEXIST = 0x1000   # 此设备不存在
ERR_LOADKERNELDLL = 0x2000    # 装载动态库失败
ERR_CMDFAILED = 0x4000        # 执行命令失败错误码
ERR_BUFFERCREATE = 0x8000     # 内存不足

# 函数调用返回状态值 (Function return status values)
STATUS_OK = 1
STATUS_ERR = 0


# 1. ZLGCAN系列接口卡信息的数据类型 (Board info structure)
class VCI_BOARD_INFO(Structure):
    """
    Board information structure.
    
    Attributes:
        hw_Version (int): Hardware version
        fw_Version (int): Firmware version
        dr_Version (int): Driver version
        in_Version (int): Interface version
        irq_Num (int): IRQ number
        can_Num (int): Number of CAN channels
        str_Serial_Num (bytes): Serial number (20 bytes)
        str_hw_Type (bytes): Hardware type description (40 bytes)
        Reserved (list): Reserved fields
    """
    _fields_ = [
        ("hw_Version", c_ushort),
        ("fw_Version", c_ushort),
        ("dr_Version", c_ushort),
        ("in_Version", c_ushort),
        ("irq_Num", c_ushort),
        ("can_Num", c_ubyte),
        ("str_Serial_Num", c_char * 20),
        ("str_hw_Type", c_char * 40),
        ("Reserved", c_ushort * 4)
    ]


# 2. 定义CAN信息帧的数据类型 (CAN frame structure)
class VCI_CAN_OBJ(Structure):
    """
    CAN frame structure.
    
    Attributes:
        ID (int): CAN identifier (11-bit: 0x000-0x7FF, 29-bit: 0x00000000-0x1FFFFFFF)
        TimeStamp (int): Timestamp in milliseconds
        TimeFlag (int): 0=no timestamp, 1=hardware timestamp available
        SendType (int): 0=normal send, 1=single send, 2=self-test send
        RemoteFlag (int): 0=data frame, 1=remote frame (RTR)
        ExternFlag (int): 0=standard frame (11-bit), 1=extended frame (29-bit)
        DataLen (int): Data length in bytes (0-8)
        Data (list): Data bytes array [0-7]
        Reserved (list): Reserved fields
    """
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte),
        ("RemoteFlag", c_ubyte),   # 是否是远程帧
        ("ExternFlag", c_ubyte),   # 是否是扩展帧
        ("DataLen", c_ubyte),
        ("Data", c_ubyte * 8),
        ("Reserved", c_ubyte * 3)
    ]


# 3. 定义CAN控制器状态的数据类型 (CAN status structure)
class VCI_CAN_STATUS(Structure):
    """
    CAN controller status structure.
    
    Attributes:
        ErrInterrupt (int): Error interrupt flag
        regMode (int): Mode register value
        regStatus (int): Status register value
        regALCapture (int): Arbitration lost capture register
        regECCapture (int): Error code capture register
        regEWLimit (int): Error warning limit register
        regRECounter (int): Receive error counter
        regTECounter (int): Transmit error counter
        Reserved (int): Reserved field
    """
    _fields_ = [
        ("ErrInterrupt", c_ubyte),
        ("regMode", c_ubyte),
        ("regStatus", c_ubyte),
        ("regALCapture", c_ubyte),
        ("regECCapture", c_ubyte),
        ("regEWLimit", c_ubyte),
        ("regRECounter", c_ubyte),
        ("regTECounter", c_ubyte),
        ("Reserved", c_uint)
    ]


# 4. 定义错误信息的数据类型 (Error info structure)
class VCI_ERR_INFO(Structure):
    """
    Error information structure.
    
    Attributes:
        ErrCode (int): Error code (see ERR_CAN_* constants)
        Passive_ErrData (list): Passive error data bytes [0-2]
        ArLost_ErrData (int): Arbitration lost error data
    """
    _fields_ = [
        ("ErrCode", c_uint),
        ("Passive_ErrData", c_ubyte * 3),
        ("ArLost_ErrData", c_ubyte)
    ]


# 5. 定义初始化CAN的数据类型 (CAN initialization config structure)
class VCI_INIT_CONFIG(Structure):
    """
    CAN initialization configuration structure.
    
    Attributes:
        AccCode (int): Acceptance code for filtering (0x00000000 = accept all)
        AccMask (int): Acceptance mask for filtering (0xFFFFFFFF = accept all)
        Reserved (int): Reserved field
        Filter (int): Filter mode (0=close filter, 1=open filter)
        Timing0 (int): Timing parameter 0 for baud rate
        Timing1 (int): Timing parameter 1 for baud rate
        Mode (int): Working mode (0=normal mode, 1=listen-only mode, 2=self-test mode)
    
    Common baud rate settings (Timing0, Timing1):
        - 5 Kbps:    (0xBF, 0xFF)
        - 10 Kbps:   (0x31, 0x1C)
        - 20 Kbps:   (0x18, 0x1C)
        - 40 Kbps:   (0x87, 0xFF)
        - 50 Kbps:   (0x09, 0x1C)
        - 80 Kbps:   (0x83, 0xFF)
        - 100 Kbps:  (0x04, 0x1C)
        - 125 Kbps:  (0x03, 0x1C)
        - 200 Kbps:  (0x81, 0xFA)
        - 250 Kbps:  (0x01, 0x1C)
        - 400 Kbps:  (0x80, 0xFA)
        - 500 Kbps:  (0x00, 0x1C)
        - 666 Kbps:  (0x80, 0xB6)
        - 800 Kbps:  (0x00, 0x16)
        - 1000 Kbps: (0x00, 0x14)
    """
    _fields_ = [
        ("AccCode", c_uint),
        ("AccMask", c_uint),
        ("Reserved", c_uint),
        ("Filter", c_ubyte),
        ("Timing0", c_ubyte),
        ("Timing1", c_ubyte),
        ("Mode", c_ubyte)
    ]


class ControlCAN:
    """
    Python wrapper class for ZLGCAN CAN interface library.
    
    This class provides a Pythonic interface to the ZLGCAN CAN device drivers,
    handling all ctypes conversions and providing type hints for better IDE support.
    
    Example:
        >>> can = ControlCAN()
        >>> can.open_device(VCI_USBCAN2, 0)
        >>> config = VCI_INIT_CONFIG()
        >>> config.Timing0 = 0x00
        >>> config.Timing1 = 0x1C  # 500 Kbps
        >>> can.init_can(VCI_USBCAN2, 0, 0, config)
        >>> can.start_can(VCI_USBCAN2, 0, 0)
        >>> # ... send/receive frames ...
        >>> can.close_device(VCI_USBCAN2, 0)
    """
    
    def __init__(self, lib_path: Optional[str] = None):
        """
        Initialize the CAN library.
        
        Args:
            lib_path: Path to the ControlCAN library file.
                     On Windows: "ControlCAN.dll"
                     On Linux: "libcontrolcan.so"
                     If None, uses default library name based on platform.
        
        Raises:
            OSError: If the library file cannot be found or loaded.
        """
        if lib_path is None:
            lib_path = "libusbcan.so"
        
        self.lib = CDLL(lib_path)
        self._setup_functions()
    
    def _setup_functions(self) -> None:
        """Setup function signatures for the CAN library."""
        
        # VCI_OpenDevice
        self.lib.VCI_OpenDevice.argtypes = [c_uint, c_uint, c_uint]
        self.lib.VCI_OpenDevice.restype = c_uint
        
        # VCI_CloseDevice
        self.lib.VCI_CloseDevice.argtypes = [c_uint, c_uint]
        self.lib.VCI_CloseDevice.restype = c_uint
        
        # VCI_InitCAN
        self.lib.VCI_InitCAN.argtypes = [c_uint, c_uint, c_uint, POINTER(VCI_INIT_CONFIG)]
        self.lib.VCI_InitCAN.restype = c_uint
        
        # VCI_ReadBoardInfo
        self.lib.VCI_ReadBoardInfo.argtypes = [c_uint, c_uint, POINTER(VCI_BOARD_INFO)]
        self.lib.VCI_ReadBoardInfo.restype = c_uint
        
        # VCI_ReadErrInfo
        self.lib.VCI_ReadErrInfo.argtypes = [c_uint, c_uint, c_uint, POINTER(VCI_ERR_INFO)]
        self.lib.VCI_ReadErrInfo.restype = c_uint
        
        # VCI_ReadCANStatus
        self.lib.VCI_ReadCANStatus.argtypes = [c_uint, c_uint, c_uint, POINTER(VCI_CAN_STATUS)]
        self.lib.VCI_ReadCANStatus.restype = c_uint
        
        # VCI_GetReference
        self.lib.VCI_GetReference.argtypes = [c_uint, c_uint, c_uint, c_uint, c_void_p]
        self.lib.VCI_GetReference.restype = c_uint
        
        # VCI_SetReference
        self.lib.VCI_SetReference.argtypes = [c_uint, c_uint, c_uint, c_uint, c_void_p]
        self.lib.VCI_SetReference.restype = c_uint
        
        # VCI_GetReceiveNum
        self.lib.VCI_GetReceiveNum.argtypes = [c_uint, c_uint, c_uint]
        self.lib.VCI_GetReceiveNum.restype = c_ulong
        
        # VCI_ClearBuffer
        self.lib.VCI_ClearBuffer.argtypes = [c_uint, c_uint, c_uint]
        self.lib.VCI_ClearBuffer.restype = c_uint
        
        # VCI_StartCAN
        self.lib.VCI_StartCAN.argtypes = [c_uint, c_uint, c_uint]
        self.lib.VCI_StartCAN.restype = c_uint
        
        # VCI_ResetCAN
        self.lib.VCI_ResetCAN.argtypes = [c_uint, c_uint, c_uint]
        self.lib.VCI_ResetCAN.restype = c_uint
        
        # VCI_Transmit
        self.lib.VCI_Transmit.argtypes = [c_uint, c_uint, c_uint, POINTER(VCI_CAN_OBJ), c_uint]
        self.lib.VCI_Transmit.restype = c_ulong
        
        # VCI_Receive
        self.lib.VCI_Receive.argtypes = [c_uint, c_uint, c_uint, POINTER(VCI_CAN_OBJ), c_uint, c_int]
        self.lib.VCI_Receive.restype = c_ulong
    
    def open_device(self, device_type: int, device_index: int, reserved: int = 0) -> int:
        """
        Open CAN device.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0 (0 for first device, 1 for second, etc.)
            reserved: Reserved parameter, must be 0
        
        Returns:
            STATUS_OK (1) if successful, STATUS_ERR (0) if failed
        
        Example:
            >>> can = ControlCAN()
            >>> result = can.open_device(VCI_USBCAN2, 0)
            >>> if result == STATUS_OK:
            ...     print("Device opened successfully")
        """
        return self.lib.VCI_OpenDevice(device_type, device_index, reserved)
    
    def close_device(self, device_type: int, device_index: int) -> int:
        """
        Close CAN device.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
        
        Returns:
            STATUS_OK (1) if successful, STATUS_ERR (0) if failed
        
        Example:
            >>> can.close_device(VCI_USBCAN2, 0)
        """
        return self.lib.VCI_CloseDevice(device_type, device_index)
    
    def init_can(self, device_type: int, device_index: int, channel_index: int, 
                 init_config: VCI_INIT_CONFIG) -> int:
        """
        Initialize CAN channel with specified configuration.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
            channel_index: CAN channel index (0 or 1, depending on device)
            init_config: Initialization configuration (VCI_INIT_CONFIG structure)
        
        Returns:
            STATUS_OK (1) if successful, STATUS_ERR (0) if failed
        
        Example:
            >>> config = VCI_INIT_CONFIG()
            >>> config.AccCode = 0x00000000    # Accept all IDs
            >>> config.AccMask = 0xFFFFFFFF    # Accept all IDs
            >>> config.Filter = 1              # Double filter
            >>> config.Timing0 = 0x00          # 500 Kbps
            >>> config.Timing1 = 0x1C          # 500 Kbps
            >>> config.Mode = 0                # Normal mode (0=normal, 1=listen-only, 2=self-test)
            >>> can.init_can(VCI_USBCAN2, 0, 0, config)
        """
        return self.lib.VCI_InitCAN(device_type, device_index, channel_index, byref(init_config))
    
    def read_board_info(self, device_type: int, device_index: int) -> Tuple[int, VCI_BOARD_INFO]:
        """
        Read board information.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
        
        Returns:
            Tuple of (result, board_info):
                - result: STATUS_OK (1) if successful, STATUS_ERR (0) if failed
                - board_info: VCI_BOARD_INFO structure containing board information
        
        Example:
            >>> result, info = can.read_board_info(VCI_USBCAN2, 0)
            >>> if result == STATUS_OK:
            ...     print(f"Hardware: {info.str_hw_Type.decode('utf-8')}")
            ...     print(f"Serial: {info.str_Serial_Num.decode('utf-8')}")
            ...     print(f"CAN Channels: {info.can_Num}")
        """
        info = VCI_BOARD_INFO()
        result = self.lib.VCI_ReadBoardInfo(device_type, device_index, byref(info))
        return result, info
    
    def read_err_info(self, device_type: int, device_index: int, channel_index: int) -> Tuple[int, VCI_ERR_INFO]:
        """
        Read error information from CAN channel.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
            channel_index: CAN channel index (0 or 1)
        
        Returns:
            Tuple of (result, err_info):
                - result: STATUS_OK (1) if successful, STATUS_ERR (0) if failed
                - err_info: VCI_ERR_INFO structure containing error information
        
        Example:
            >>> result, err_info = can.read_err_info(VCI_USBCAN2, 0, 0)
            >>> if result == STATUS_OK:
            ...     if err_info.ErrCode & ERR_CAN_OVERFLOW:
            ...         print("CAN FIFO overflow detected")
            ...     if err_info.ErrCode & ERR_CAN_BUSERR:
            ...         print("CAN bus error detected")
        """
        err_info = VCI_ERR_INFO()
        result = self.lib.VCI_ReadErrInfo(device_type, device_index, channel_index, byref(err_info))
        return result, err_info
    
    def read_can_status(self, device_type: int, device_index: int, channel_index: int) -> Tuple[int, VCI_CAN_STATUS]:
        """
        Read CAN controller status.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
            channel_index: CAN channel index (0 or 1)
        
        Returns:
            Tuple of (result, status):
                - result: STATUS_OK (1) if successful, STATUS_ERR (0) if failed
                - status: VCI_CAN_STATUS structure containing controller status
        
        Example:
            >>> result, status = can.read_can_status(VCI_USBCAN2, 0, 0)
            >>> if result == STATUS_OK:
            ...     print(f"RX Errors: {status.regRECounter}")
            ...     print(f"TX Errors: {status.regTECounter}")
            ...     print(f"Status: 0x{status.regStatus:02X}")
        """
        status = VCI_CAN_STATUS()
        result = self.lib.VCI_ReadCANStatus(device_type, device_index, channel_index, byref(status))
        return result, status
    
    def get_receive_num(self, device_type: int, device_index: int, channel_index: int) -> int:
        """
        Get the number of frames in the receive buffer.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
            channel_index: CAN channel index (0 or 1)
        
        Returns:
            Number of frames currently in the receive buffer
        
        Example:
            >>> num_frames = can.get_receive_num(VCI_USBCAN2, 0, 0)
            >>> print(f"Frames waiting in buffer: {num_frames}")
        """
        return self.lib.VCI_GetReceiveNum(device_type, device_index, channel_index)
    
    def clear_buffer(self, device_type: int, device_index: int, channel_index: int) -> int:
        """
        Clear the receive buffer.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
            channel_index: CAN channel index (0 or 1)
        
        Returns:
            STATUS_OK (1) if successful, STATUS_ERR (0) if failed
        
        Example:
            >>> can.clear_buffer(VCI_USBCAN2, 0, 0)
            >>> print("Receive buffer cleared")
        """
        return self.lib.VCI_ClearBuffer(device_type, device_index, channel_index)
    
    def start_can(self, device_type: int, device_index: int, channel_index: int) -> int:
        """
        Start CAN communication.
        
        Must be called after init_can() and before transmit() or receive().
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
            channel_index: CAN channel index (0 or 1)
        
        Returns:
            STATUS_OK (1) if successful, STATUS_ERR (0) if failed
        
        Example:
            >>> can.init_can(VCI_USBCAN2, 0, 0, config)
            >>> can.start_can(VCI_USBCAN2, 0, 0)
            >>> # Now ready to send/receive
        """
        return self.lib.VCI_StartCAN(device_type, device_index, channel_index)
    
    def reset_can(self, device_type: int, device_index: int, channel_index: int) -> int:
        """
        Reset CAN controller (stops communication).
        
        After reset, you must call start_can() again to resume communication.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
            channel_index: CAN channel index (0 or 1)
        
        Returns:
            STATUS_OK (1) if successful, STATUS_ERR (0) if failed
        
        Example:
            >>> can.reset_can(VCI_USBCAN2, 0, 0)
            >>> # CAN is now stopped
            >>> can.start_can(VCI_USBCAN2, 0, 0)
            >>> # CAN restarted
        """
        return self.lib.VCI_ResetCAN(device_type, device_index, channel_index)
    
    def transmit(self, device_type: int, device_index: int, channel_index: int, 
                 can_objs: Union[VCI_CAN_OBJ, List[VCI_CAN_OBJ]]) -> int:
        """
        Transmit CAN frame(s).
        
        This method handles conversion of Python VCI_CAN_OBJ structure(s) to
        C-compatible array format before calling the underlying C library.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
            channel_index: CAN channel index (0 or 1)
            can_objs: Single VCI_CAN_OBJ or list of VCI_CAN_OBJ structures to transmit
        
        Returns:
            Number of frames successfully transmitted (0 if all failed)
        
        Example (single frame):
            >>> frame = VCI_CAN_OBJ()
            >>> frame.ID = 0x123
            >>> frame.ExternFlag = 0       # Standard frame
            >>> frame.RemoteFlag = 0       # Data frame
            >>> frame.DataLen = 8
            >>> frame.Data[0] = 0x01
            >>> # ... set other data bytes ...
            >>> sent = can.transmit(VCI_USBCAN2, 0, 0, frame)
            >>> print(f"Sent {sent} frame(s)")
        
        Example (multiple frames):
            >>> frames = [frame1, frame2, frame3]
            >>> sent = can.transmit(VCI_USBCAN2, 0, 0, frames)
            >>> print(f"Sent {sent} out of {len(frames)} frame(s)")
        
        Note:
            The method creates a C-compatible array internally:
            1. Converts single frame to list if needed
            2. Creates C array type: VCI_CAN_OBJ * length
            3. Copies Python structures to C array
            4. Passes C array pointer to the library
        """
        if not isinstance(can_objs, list):
            can_objs = [can_objs]
        
        # Create C array from Python list
        array_type = VCI_CAN_OBJ * len(can_objs)
        frames = array_type(*can_objs)
        return self.lib.VCI_Transmit(device_type, device_index, channel_index, frames, len(can_objs))
    
    def receive(self, device_type: int, device_index: int, channel_index: int, 
                count: int, wait_time: int = -1) -> Tuple[int, List[VCI_CAN_OBJ]]:
        """
        Receive CAN frame(s) from the buffer.
        
        Args:
            device_type: Device type constant (e.g., VCI_USBCAN2, VCI_PCI5121)
            device_index: Device index, starting from 0
            channel_index: CAN channel index (0 or 1)
            count: Maximum number of frames to receive
            wait_time: Wait time in milliseconds:
                      -1 = wait indefinitely until frames arrive
                       0 = return immediately (non-blocking)
                      >0 = wait specified milliseconds
        
        Returns:
            Tuple of (num_received, frames):
                - num_received: Number of frames actually received
                - frames: List of VCI_CAN_OBJ structures (length = num_received)
        
        Example (blocking):
            >>> # Wait up to 1000ms for frames
            >>> count, frames = can.receive(VCI_USBCAN2, 0, 0, 10, wait_time=1000)
            >>> for frame in frames:
            ...     print(f"ID: 0x{frame.ID:03X}, Data: {list(frame.Data[:frame.DataLen])}")
        
        Example (non-blocking):
            >>> # Check immediately without waiting
            >>> count, frames = can.receive(VCI_USBCAN2, 0, 0, 10, wait_time=0)
            >>> if count > 0:
            ...     print(f"Received {count} frame(s)")
        
        Example (wait forever):
            >>> # Block until at least one frame arrives
            >>> count, frames = can.receive(VCI_USBCAN2, 0, 0, 1, wait_time=-1)
            >>> # Will return when frame arrives
        
        Note:
            - Always check get_receive_num() first to know buffer size
            - Requesting more frames than available is safe (returns what's available)
            - Use wait_time=0 in loops to avoid blocking
        """
        array_type = VCI_CAN_OBJ * count
        frames = array_type()
        result = self.lib.VCI_Receive(device_type, device_index, channel_index, frames, count, wait_time)
        return result, list(frames[:result])


if __name__ == "__main__":
    # Create CAN interface
    can = ControlCAN()
    device = (VCI_USBCAN1, 0)

    # Open device
    can.open_device(*device, 0)

    # Initialize CAN
    config = VCI_INIT_CONFIG()
    config.AccCode = 0
    config.AccMask = 0xFFFFFFFF
    config.Reserved = 0
    config.Filter = 1
    config.Timing0 = 0x00
    config.Timing1 = 0x1C
    config.Mode = 0
    can.init_can(*device, 0, config)

    # Start CAN
    can.start_can(*device, 0)

    time.sleep(5)

    # Receive frames
    # result, frames = can.receive(*device, 0, 10)

    can.close_device(*device)

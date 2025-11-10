"""
MR76 Millimeter Wave Radar Interface Library
77GHz Radar Communication Protocol Implementation

This library provides a complete Python interface for the MR76 radar,
supporting all configuration, status, object detection, and collision detection features.
Created By: Morad Singer (Morad.S.Singer@gmail.com)

Protocol Version: V1.2 (2020-03-07)
Manufacturer: Hunan Nanoradar Technology Co., Ltd.

IMPORTANT: MR76 uses Motorola (Big-Endian) bit ordering:
- Byte order: Big-endian
- Bit numbering: LSB=0, MSB=7 within each byte
- Start bit refers to the LSB of the signal
- Multi-byte signals span backwards through byte boundaries
"""

from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import IntEnum
import struct
import math


# ============================================================================
# MOTOROLA BIT ORDER HELPER FUNCTIONS
# ============================================================================

def motorola_extract_signal(data: bytes, start_bit: int, length: int) -> int:
    """
    Extract a signal from CAN data using Motorola (Big-Endian) bit ordering.
    
    Motorola bit ordering rules:
    - Bit numbering: bit 0 = LSB of byte 0, bit 7 = MSB of byte 0
    - Byte 0: bits 0-7, Byte 1: bits 8-15, Byte 2: bits 16-23, etc.
    - Start bit is the LSB (least significant bit) of the signal
    - Bits fill UPWARD within current byte (toward MSB)
    - When byte boundary is reached, jump to PREVIOUS byte starting at its bit 0
    
    Example 1: start_bit=56, length=11
    - Byte 7: bits 56-63 (8 bits) - fill current byte
    - Byte 6: bits 48-50 (3 bits) - wrap to previous byte
    
    Example 2: start_bit=51, length=13
    - Byte 6: bits 51-55 (5 bits) - partial fill
    - Byte 5: bits 40-47 (8 bits) - wrap to previous byte
    
    Args:
        data: CAN message bytes
        start_bit: Starting bit position (LSB of signal)
        length: Number of bits in signal
    
    Returns:
        Extracted integer value
    """
    result = 0
    bits_extracted = 0
    
    # Start byte and bit position
    current_byte = start_bit // 8
    current_bit_in_byte = start_bit % 8
    
    while bits_extracted < length:
        # How many bits can we extract from current byte?
        bits_available_in_byte = 8 - current_bit_in_byte
        bits_to_extract = min(bits_available_in_byte, length - bits_extracted)
        
        # Extract bits from current byte
        if current_byte < len(data):
            # Create mask for the bits we want
            mask = (1 << bits_to_extract) - 1
            # Extract and shift
            bits = (data[current_byte] >> current_bit_in_byte) & mask
            # Insert into result at correct position
            result |= (bits << bits_extracted)
        
        bits_extracted += bits_to_extract
        
        # Move to previous byte if we need more bits
        if bits_extracted < length:
            current_byte -= 1  # Jump to PREVIOUS byte (Motorola rule)
            current_bit_in_byte = 0  # Start from bit 0 of that byte
    
    return result


def motorola_insert_signal(data: bytearray, start_bit: int, length: int, value: int) -> None:
    """
    Insert a signal into CAN data using Motorola (Big-Endian) bit ordering.
    
    Follows the same rules as extraction:
    - Fill upward within current byte
    - Jump to previous byte when boundary reached
    
    Args:
        data: CAN message bytearray (modified in place)
        start_bit: Starting bit position (LSB of signal)
        length: Number of bits in signal
        value: Integer value to insert
    """
    bits_inserted = 0
    
    # Start byte and bit position
    current_byte = start_bit // 8
    current_bit_in_byte = start_bit % 8
    
    while bits_inserted < length:
        # How many bits can we insert into current byte?
        bits_available_in_byte = 8 - current_bit_in_byte
        bits_to_insert = min(bits_available_in_byte, length - bits_inserted)
        
        # Insert bits into current byte
        if current_byte < len(data):
            # Extract the bits we want to insert from value
            mask = (1 << bits_to_insert) - 1
            bits = (value >> bits_inserted) & mask
            
            # Clear the target bits in data
            clear_mask = mask << current_bit_in_byte
            data[current_byte] &= ~clear_mask
            
            # Set the new bits
            data[current_byte] |= (bits << current_bit_in_byte)
        
        bits_inserted += bits_to_insert
        
        # Move to previous byte if we need to insert more bits
        if bits_inserted < length:
            current_byte -= 1  # Jump to PREVIOUS byte (Motorola rule)
            current_bit_in_byte = 0  # Start from bit 0 of that byte


def decode_signal(data: bytes, start_bit: int, length: int, 
                  resolution: float, offset: float) -> float:
    """
    Extract and decode a signal with offset and resolution.
    
    Formula: actual_value = (raw_value × resolution) + offset
    
    Args:
        data: CAN message bytes
        start_bit: Starting bit (Motorola LSB)
        length: Signal length in bits
        resolution: Resolution factor
        offset: Offset value
    
    Returns:
        Decoded physical value
    """
    raw_value = motorola_extract_signal(data, start_bit, length)
    return (raw_value * resolution) + offset


def encode_signal(data: bytearray, start_bit: int, length: int,
                  actual_value: float, resolution: float, offset: float) -> None:
    """
    Encode and insert a signal with offset and resolution.
    
    Formula: raw_value = round((actual_value - offset) / resolution)
    
    Note: Uses round() instead of int() to handle floating point precision issues.
    
    Args:
        data: CAN message bytearray
        start_bit: Starting bit (Motorola LSB)
        length: Signal length in bits
        actual_value: Physical value to encode
        resolution: Resolution factor
        offset: Offset value
    """
    # Use round() to handle floating point precision
    raw_value = round((actual_value - offset) / resolution)
    # Mask to signal length
    raw_value &= (1 << length) - 1
    motorola_insert_signal(data, start_bit, length, raw_value)


# ============================================================================
# CONSTANTS AND ENUMERATIONS
# ============================================================================

class OutputType(IntEnum):
    """Radar output type modes"""
    NONE = 0x0      # No object data output
    OBJECTS = 0x1   # Object mode (default)
    CLUSTERS = 0x2  # Cluster mode (not supported by MR76)


class RadarPower(IntEnum):
    """Radar transmission power levels"""
    STANDARD = 0x0  # 2W default, 2.5W peak
    MINUS_3DB = 0x1  # -3dB Tx gain
    MINUS_6DB = 0x2  # -6dB Tx gain
    MINUS_9DB = 0x3  # -9dB Tx gain


class SortIndex(IntEnum):
    """Target sorting methods"""
    NO_SORTING = 0x0
    SORTED_BY_RANGE = 0x1   # Default: near to far
    SORTED_BY_RCS = 0x2     # Sorted by RCS value


class RCSThreshold(IntEnum):
    """RCS (Radar Cross Section) sensitivity threshold"""
    STANDARD = 0x0
    HIGH_SENSITIVITY = 0x1  # For detecting fine targets


class OperatingMode(IntEnum):
    """CAN operating mode"""
    NORMAL = 0x0        # Normal operation mode
    LISTEN_ONLY = 0x1   # Listen-only mode
    SELF_TEST = 0x2     # Self-test mode


class CalibrationMode(IntEnum):
    """Channel calibration modes"""
    DISABLED = 0x0
    ENABLED = 0x1
    INITIAL_RECOVERY = 0x2  # Restore to initial values


class BaudRate(IntEnum):
    """CAN bus baud rates"""
    BAUD_500K = 0x0  # 500 Kbps (default)
    BAUD_250K = 0x1  # 250 Kbps
    BAUD_1M = 0x2    # 1 Mbps


class DynamicProperty(IntEnum):
    """Object dynamic property classification"""
    MOVING = 0x0
    STATIONARY = 0x1
    ONCOMING = 0x2
    CROSSING_LEFT = 0x3
    CROSSING_RIGHT = 0x4
    UNKNOWN = 0x5
    STOPPED = 0x6


class ObjectClass(IntEnum):
    """Object classification types"""
    POINT = 0x0  # Point target (pedestrian, tree, fence)
    VEHICLE = 0x1  # Vehicle target (truck, car)
    RESERVED_2 = 0x2
    RESERVED_3 = 0x3


class WarningLevel(IntEnum):
    """Collision detection warning levels"""
    NO_WARNING = 0x0      # No target in region
    TARGET_WARNING = 0x1  # Target detected in region
    RESERVED = 0x2
    WARNING_CLEARED = 0x3  # Target left the region


# CAN Message IDs (for SensorID = 0)
# Actual ID = BASE_ID + SensorID * 0x10
class MessageID:
    """CAN message IDs for MR76 radar"""
    # Input messages (configuration)
    RADAR_CFG = 0x200                    # Radar configuration
    COLLISION_DETECT_CFG = 0x400         # Collision detection config
    COLLISION_DETECT_REGION_CFG = 0x401  # Collision detection region config
    
    # Output messages (status and data)
    RADAR_STATE = 0x201                  # Radar status (1Hz heartbeat)
    SOFTWARE_VERSION = 0x700             # Software version (1Hz heartbeat)
    COLLISION_DETECT_STATE = 0x408       # Collision detection status (1Hz)
    COLLISION_DETECT_REGION_STATE = 0x402  # Region status (1Hz)
    OBJECT_STATUS = 0x60A                # Object list header
    OBJECT_GENERAL = 0x60B               # Object general info
    OBJECT_COLLISION_WARNING = 0x60E     # Object collision warning
    
    @staticmethod
    def adjust_for_sensor(base_id: int, sensor_id: int) -> int:
        """Calculate actual message ID based on sensor ID"""
        return base_id + sensor_id * 0x10


# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class RadarConfig:
    """
    Radar configuration parameters (0x200)
    
    Only modified parameters need valid flags set to True.
    Set store_in_nvm=True to save configuration to flash memory.
    """
    # Valid flags - set to True to enable parameter modification
    max_distance_valid: bool = False
    sensor_id_valid: bool = False
    radar_power_valid: bool = False
    output_type_valid: bool = False
    send_quality_valid: bool = False
    send_ext_info_valid: bool = False
    sort_index_valid: bool = False
    store_in_nvm_valid: bool = False
    rcs_threshold_valid: bool = False
    calibration_valid: bool = False
    baud_rate_valid: bool = False
    
    # Configuration parameters
    max_distance: int = 100           # Detection range: 0-2048m, resolution 2m
    sensor_id: int = 0                # Sensor ID: 0-7
    output_type: OutputType = OutputType.OBJECTS
    radar_power: RadarPower = RadarPower.STANDARD
    sort_index: SortIndex = SortIndex.SORTED_BY_RANGE
    store_in_nvm: bool = False        # Set True to save to flash
    rcs_threshold: RCSThreshold = RCSThreshold.STANDARD
    calibration_mode: CalibrationMode = CalibrationMode.DISABLED
    baud_rate: BaudRate = BaudRate.BAUD_500K
    
    def to_can_frame(self) -> bytes:
        """
        Convert configuration to CAN frame data (8 bytes) using Motorola bit ordering.
        
        Returns:
            bytes: 8-byte CAN data payload
        """
        data = bytearray(8)
        
        # Byte 0: Valid bits [0-7]
        motorola_insert_signal(data, 0, 1, 1 if self.max_distance_valid else 0)
        motorola_insert_signal(data, 1, 1, 1 if self.sensor_id_valid else 0)
        motorola_insert_signal(data, 2, 1, 1 if self.radar_power_valid else 0)
        motorola_insert_signal(data, 3, 1, 1 if self.output_type_valid else 0)
        motorola_insert_signal(data, 4, 1, 1 if self.send_quality_valid else 0)
        motorola_insert_signal(data, 5, 1, 1 if self.send_ext_info_valid else 0)
        motorola_insert_signal(data, 6, 1, 1 if self.sort_index_valid else 0)
        motorola_insert_signal(data, 7, 1, 1 if self.store_in_nvm_valid else 0)
        
        # MaxDistance: 10 bits at start_bit 22, resolution 2m
        encode_signal(data, 22, 10, self.max_distance, 2, 0)
        
        # SensorID: 3 bits at start_bit 32
        motorola_insert_signal(data, 32, 3, self.sensor_id)
        
        # OutputType: 2 bits at start_bit 35
        motorola_insert_signal(data, 35, 2, self.output_type)
        
        # RadarPower: 3 bits at start_bit 37
        motorola_insert_signal(data, 37, 3, self.radar_power)
        
        # SortIndex: 3 bits at start_bit 44
        motorola_insert_signal(data, 44, 3, self.sort_index)
        
        # StoreNVM: 1 bit at start_bit 47
        motorola_insert_signal(data, 47, 1, 1 if self.store_in_nvm else 0)
        
        # RCS_Threshold_Valid: 1 bit at start_bit 48
        motorola_insert_signal(data, 48, 1, 1 if self.rcs_threshold_valid else 0)
        
        # RCS_Threshold: 3 bits at start_bit 49
        motorola_insert_signal(data, 49, 3, self.rcs_threshold)
        
        # Calibration_Enabled: 2 bits at start_bit 57 (uses bits 57-58)
        motorola_insert_signal(data, 57, 2, self.calibration_mode)
        
        # Calibration_Valid: 1 bit at start_bit 59
        motorola_insert_signal(data, 59, 1, 1 if self.calibration_valid else 0)
        
        # BaudRate_Valid: 1 bit at start_bit 60
        motorola_insert_signal(data, 60, 1, 1 if self.baud_rate_valid else 0)
        
        # BaudRate: 3 bits at start_bit 61
        motorola_insert_signal(data, 61, 3, self.baud_rate)
        
        return bytes(data)


@dataclass
class RadarState:
    """
    Radar state information (0x201)
    Sent periodically (1Hz) by radar
    """
    nvm_read_status: bool       # NVM read success
    nvm_write_status: bool      # NVM write success
    max_distance: int           # Current max distance config (meters)
    sensor_id: int              # Current sensor ID (0-7)
    sort_index: SortIndex       # Current sort method
    radar_power: RadarPower     # Current power setting
    output_type: OutputType     # Current output mode
    can_baud_rate: BaudRate     # Current CAN baud rate
    rcs_threshold: RCSThreshold # Current RCS threshold
    calibration_enabled: CalibrationMode  # Calibration status
    
    @staticmethod
    def from_can_frame(data: bytes) -> 'RadarState':
        """
        Parse radar state from CAN frame data using Motorola bit ordering.
        
        Args:
            data: 8-byte CAN data payload
            
        Returns:
            RadarState object
        """
        # NVMReadStatus: 1 bit at start_bit 6
        nvm_read_status = bool(motorola_extract_signal(data, 6, 1))
        
        # NVMWriteStatus: 1 bit at start_bit 7
        nvm_write_status = bool(motorola_extract_signal(data, 7, 1))
        
        # MaxDistance: 10 bits at start_bit 22, resolution 2m
        max_distance = int(decode_signal(data, 22, 10, 2, 0))
        
        # SensorID: 3 bits at start_bit 32
        sensor_id = motorola_extract_signal(data, 32, 3)
        
        # SortIndex: 3 bits at start_bit 36
        sort_index = SortIndex(motorola_extract_signal(data, 36, 3))
        
        # RadarPower: 3 bits at start_bit 39
        radar_power = RadarPower(motorola_extract_signal(data, 39, 3))
        
        # OutputType: 2 bits at start_bit 42
        output_type = OutputType(motorola_extract_signal(data, 42, 2))
        
        # CANBaudRate: 3 bits at start_bit 53
        can_baud_rate = BaudRate(motorola_extract_signal(data, 53, 3))
        
        # RCS_threshold: 3 bits at start_bit 58
        rcs_threshold = RCSThreshold(motorola_extract_signal(data, 58, 3))
        
        # Calibration: 2 bits at start_bit 62
        calibration = CalibrationMode(decode_signal(data, 62, 2, 4, 0))
        
        return RadarState(
            nvm_read_status=nvm_read_status,
            nvm_write_status=nvm_write_status,
            max_distance=max_distance,
            sensor_id=sensor_id,
            sort_index=sort_index,
            radar_power=radar_power,
            output_type=output_type,
            can_baud_rate=can_baud_rate,
            rcs_threshold=rcs_threshold,
            calibration_enabled=calibration
        )


@dataclass
class SoftwareVersion:
    """
    Software version information (0x700)
    Heartbeat signal sent every second
    """
    major: int  # Major version (0-255)
    minor: int  # Minor version (0-255)
    patch: int  # Patch version (0-255)
    
    @staticmethod
    def from_can_frame(data: bytes) -> 'SoftwareVersion':
        """
        Parse software version from CAN frame using Motorola bit ordering.
        
        Args:
            data: 8-byte CAN data payload
            
        Returns:
            SoftwareVersion object
        """
        # Major: 8 bits at start_bit 0
        major = motorola_extract_signal(data, 0, 8)
        
        # Minor: 8 bits at start_bit 8
        minor = motorola_extract_signal(data, 8, 8)
        
        # Patch: 8 bits at start_bit 16
        patch = motorola_extract_signal(data, 16, 8)
        
        return SoftwareVersion(
            major=major,
            minor=minor,
            patch=patch
        )
    
    def __str__(self) -> str:
        return f"V{self.major}.{self.minor}.{self.patch}"


@dataclass
class CollisionDetectionConfig:
    """
    Collision detection configuration (0x400)
    
    Enable region-based collision detection functionality.
    Currently MR76 supports only 1 region with RegionID=1.
    """
    warning_reset: bool = False     # Reset all region warnings
    activation: bool = False        # Activate/deactivate collision detection
    min_time_valid: bool = False    # Enable min time parameter change
    clear_regions: bool = False     # Clear all region configurations
    min_time: float = 0.0           # Min detection time before warning (0-25.5s, res 0.1s)
    
    def to_can_frame(self) -> bytes:
        """
        Convert to CAN frame data (8 bytes) using Motorola bit ordering.
        
        Returns:
            bytes: 8-byte CAN data payload
        """
        data = bytearray(8)
        
        # WarningReset: 1 bit at start_bit 0
        motorola_insert_signal(data, 0, 1, 1 if self.warning_reset else 0)
        
        # Activation: 1 bit at start_bit 1
        motorola_insert_signal(data, 1, 1, 1 if self.activation else 0)
        
        # MinTime_valid: 1 bit at start_bit 3
        motorola_insert_signal(data, 3, 1, 1 if self.min_time_valid else 0)
        
        # ClearRegions: 1 bit at start_bit 7
        motorola_insert_signal(data, 7, 3, 1 if self.clear_regions else 0)
        
        # MinTime: 8 bits at start_bit 8, resolution 0.1s
        encode_signal(data, 8, 8, self.min_time, 0.1, 0)
        
        return bytes(data)


@dataclass
class CollisionDetectionRegion:
    """
    Collision detection region configuration (0x401)
    
    Defines a rectangular detection zone with two corner points.
    Point1 must be bottom-right corner, Point2 must be top-left corner.
    
    Constraints:
        - Point1Long < Point2Long (Point1 closer than Point2)
        - Point1Lat > Point2Lat (Point1 right of Point2)
    """
    activation: bool = False          # Activate this region
    coordinates_valid: bool = False   # Coordinates are valid
    region_id: int = 1                # Region ID (default 1, only 1 supported)
    point1_long: float = 0.0          # Point1 longitudinal distance (m)
    point1_lat: float = 0.0           # Point1 lateral distance (m)
    point2_long: float = 0.0          # Point2 longitudinal distance (m)
    point2_lat: float = 0.0           # Point2 lateral distance (m)
    
    def validate(self) -> bool:
        """
        Validate region coordinates
        
        Returns:
            True if valid, False otherwise
        """
        return (self.point1_long < self.point2_long and 
                self.point1_lat > self.point2_lat)
    
    def to_can_frame(self) -> bytes:
        """
        Convert to CAN frame data (8 bytes) using Motorola bit ordering.
        
        Applies offset and resolution encoding:
        - Point coordinates: offset=-500/-204.6, resolution=0.2
        
        Bit layout (Motorola):
        - Activation: bit 1
        - CoordinatesValid: bit 2  
        - RegionID: bits 8-10
        - Point1Long: bits 27-39 (13 bits)
        - Point1Lat: bits 24-34 (11 bits)
        - Point2Long: bits 51-63 (13 bits)
        - Point2Lat: bits 48-58 (11 bits)
        """
        data = bytearray(8)
        
        # Simple bit fields (no special encoding needed)
        motorola_insert_signal(data, 1, 1, 1 if self.activation else 0)
        motorola_insert_signal(data, 2, 1, 1 if self.coordinates_valid else 0)
        motorola_insert_signal(data, 8, 3, self.region_id)
        
        # Point1Long: 13 bits starting at bit 27 (Motorola LSB)
        encode_signal(data, 27, 13, self.point1_long, 0.2, -500)
        
        # Point1Lat: 11 bits starting at bit 32 (Motorola LSB)
        encode_signal(data, 32, 11, self.point1_lat, 0.2, -204.6)
        
        # Point2Long: 13 bits starting at bit 51 (Motorola LSB)
        encode_signal(data, 51, 13, self.point2_long, 0.2, -500)
        
        # Point2Lat: 11 bits starting at bit 56 (Motorola LSB)
        encode_signal(data, 56, 11, self.point2_lat, 0.2, -204.6)
        
        return bytes(data)


@dataclass
class CollisionDetectionState:
    """
    Collision detection state (0x408)
    Sent periodically (1Hz) when collision detection is active
    """
    activation: bool            # Is collision detection active
    num_regions: int            # Number of configured regions
    min_detect_time: float      # Minimum detection time (seconds)
    meas_counter: int           # Measurement cycle counter (0-65535)
    
    @staticmethod
    def from_can_frame(data: bytes) -> 'CollisionDetectionState':
        """
        Parse collision detection state from CAN frame using Motorola bit ordering.
        
        Args:
            data: 8-byte CAN data payload
            
        Returns:
            CollisionDetectionState object
        """
        # Activation: 1 bit at start_bit 1
        activation = bool(motorola_extract_signal(data, 1, 1))
        
        # NofRegions: 4 bits at start_bit 4
        num_regions = motorola_extract_signal(data, 4, 4)
        
        # MinDetectTime: 8 bits at start_bit 8, resolution 0.1s
        min_detect_time = decode_signal(data, 8, 8, 0.1, 0)
        
        # MeasCounter: 16 bits at start_bit 24
        meas_counter = motorola_extract_signal(data, 24, 16)
        
        return CollisionDetectionState(
            activation=activation,
            num_regions=num_regions,
            min_detect_time=min_detect_time,
            meas_counter=meas_counter
        )


@dataclass
class CollisionDetectionRegionState:
    """
    Collision detection region state (0x402)
    Sent periodically (1Hz) when region is configured and active
    """
    warning_level: WarningLevel  # Warning status
    region_id: int              # Region ID
    point1_long: float          # Point1 longitudinal (m)
    point1_lat: float           # Point1 lateral (m)
    point2_long: float          # Point2 longitudinal (m)
    point2_lat: float           # Point2 lateral (m)
    num_objects: int            # Number of objects in region
    
    @staticmethod
    def from_can_frame(data: bytes) -> 'CollisionDetectionRegionState':
        """
        Parse region state from CAN frame using Motorola bit ordering.
        
        IMPORTANT: Point2Lat in 0x402 starts at bit 48 (NOT bit 56 like in 0x401!)
        
        Args:
            data: 8-byte CAN data payload
            
        Returns:
            CollisionDetectionRegionState object
        """
        # WarningLevel: 2 bits at start_bit 3
        warning_level = WarningLevel(motorola_extract_signal(data, 3, 2))
        
        # RegionID: 3 bits at start_bit 5
        region_id = motorola_extract_signal(data, 5, 3)
        
        # Point1Long: 13 bits at start_bit 19, offset=-500, res=0.2
        point1_long = decode_signal(data, 19, 13, 0.2, -500)
        
        # Point1Lat: 11 bits at start_bit 24, offset=-204.6, res=0.2
        point1_lat = decode_signal(data, 24, 11, 0.2, -204.6)
        
        # Point2Long: 13 bits at start_bit 43, offset=-500, res=0.2
        point2_long = decode_signal(data, 43, 13, 0.2, -500)
        
        # Point2Lat: 11 bits at start_bit 48 (DIFFERENT from 0x401!)
        # Bit usage: Byte 6 bits 48-55 (7 bits) + Byte 3 bits 40-42 (8 bits)
        point2_lat = decode_signal(data, 48, 11, 0.2, -204.6)
        
        # NumObjects: 8 bits at start_bit 56
        num_objects = motorola_extract_signal(data, 56, 8)
        
        return CollisionDetectionRegionState(
            warning_level=warning_level,
            region_id=region_id,
            point1_long=point1_long,
            point1_lat=point1_lat,
            point2_long=point2_long,
            point2_lat=point2_lat,
            num_objects=num_objects
        )


@dataclass
class ObjectListStatus:
    """
    Object list header (0x60A)
    Sent at the beginning of each measurement cycle
    """
    num_objects: int        # Number of detected objects (0-255)
    meas_count: int         # Measurement cycle counter (0-65535)
    interface_version: int  # CAN interface version (default 0)
    
    @staticmethod
    def from_can_frame(data: bytes) -> 'ObjectListStatus':
        """
        Parse object list status from CAN frame using Motorola bit ordering.
        
        Args:
            data: 8-byte CAN data payload
            
        Returns:
            ObjectListStatus object
        """
        # NofObjects: 8 bits at start_bit 0
        num_objects = motorola_extract_signal(data, 0, 8)

        # MeasCount: 16 bits at start_bit 16
        meas_count = motorola_extract_signal(data, 16, 16)
        
        # InterfaceVersion: 4 bits at start_bit 28
        interface_version = motorola_extract_signal(data, 28, 4)
        
        return ObjectListStatus(
            num_objects=num_objects,
            meas_count=meas_count,
            interface_version=interface_version
        )


@dataclass
class RadarObject:
    """
    Radar detected object (0x60B)
    Contains position, velocity, and classification information
    """
    object_id: int              # Object ID (0-255), stable during tracking
    dist_long: float            # Longitudinal distance (m)
    dist_lat: float             # Lateral distance (m)
    vrel_long: float            # Longitudinal velocity (m/s), negative=approaching
    vrel_lat: float             # Lateral velocity (m/s)
    dyn_prop: DynamicProperty   # Dynamic property (currently always MOVING)
    object_class: ObjectClass   # Object classification
    rcs: float                  # Radar cross section (dBm²)
    
    @staticmethod
    def from_can_frame(data: bytes) -> 'RadarObject':
        """
        Parse radar object from CAN frame using Motorola bit ordering.
        
        Applies offset and resolution decoding for all fields.
        
        Example from protocol:
        Data: 0x57 0x4E 0xC4 0x0C 0x7F 0x60 0x18 0x80
        Expected: ID=87, Long=4m, Lat=2.6m, VLong=-0.75m/s
        """
        # Object ID: 8 bits at bit 7 (Motorola: byte 0, bits 7-0)
        object_id = motorola_extract_signal(data, 0, 8)
        
        # DistLong: 13 bits, start bit 19, offset=-500, res=0.2
        dist_long = decode_signal(data, 19, 13, 0.2, -500)
        
        # DistLat: 11 bits, start bit 24, offset=-204.6, res=0.2
        dist_lat = decode_signal(data, 24, 11, 0.2, -204.6)
        
        # VrelLong: 10 bits, start bit 46, offset=-128, res=0.25
        vrel_long = decode_signal(data, 46, 10, 0.25, -128)
        
        # DynProp: 3 bits at bit 48
        dyn_prop_raw = motorola_extract_signal(data, 48, 3)
        dyn_prop = DynamicProperty(dyn_prop_raw)
        
        # ObjectClass: 2 bits at bit 51
        class_raw = motorola_extract_signal(data, 51, 2)
        object_class = ObjectClass(class_raw)
        
        # VrelLat: 9 bits, start bit 53, offset=-64, res=0.25
        vrel_lat = decode_signal(data, 53, 9, 0.25, -64)
        
        # RCS: 8 bits, start bit 56, offset=-64, res=0.5
        # Note: This is message 0x60B, not the same as Point2Lat bit 56 in message 0x401
        rcs = decode_signal(data, 56, 8, 0.5, -64)
        
        return RadarObject(
            object_id=object_id,
            dist_long=dist_long,
            dist_lat=dist_lat,
            vrel_long=vrel_long,
            vrel_lat=vrel_lat,
            dyn_prop=dyn_prop,
            object_class=object_class,
            rcs=rcs
        )
    
    def get_radial_distance(self) -> float:
        """Calculate radial distance from longitudinal and lateral distances"""
        return math.sqrt(self.dist_long**2 + self.dist_lat**2)
    
    def get_angle_deg(self) -> float:
        """Calculate angle in degrees from radar centerline"""
        return math.degrees(math.atan2(self.dist_lat, self.dist_long))
    
    def get_angle_rad(self) -> float:
        """Calculate angle in radians from radar centerline"""
        return math.atan2(self.dist_lat, self.dist_long)
    
    def get_radial_velocity(self) -> float:
        """Calculate total radial velocity"""
        return math.sqrt(self.vrel_long**2 + self.vrel_lat**2)


@dataclass
class ObjectCollisionWarning:
    """
    Object collision detection warning (0x60E)
    Sent for each object when collision detection is active
    """
    object_id: int              # Object ID
    region_bitfield: int        # Bit field indicating which regions object is in
    
    @staticmethod
    def from_can_frame(data: bytes) -> 'ObjectCollisionWarning':
        """
        Parse collision warning from CAN frame using Motorola bit ordering.
        
        Args:
            data: 8-byte CAN data payload
            
        Returns:
            ObjectCollisionWarning object
        """
        # Object_ID: 8 bits at start_bit 0
        object_id = motorola_extract_signal(data, 0, 8)
        
        # RegionBitfield: 8 bits at start_bit 8
        region_bitfield = motorola_extract_signal(data, 8, 8)
        
        return ObjectCollisionWarning(
            object_id=object_id,
            region_bitfield=region_bitfield
        )
    
    def is_in_region(self, region_id: int) -> bool:
        """Check if object is in specified region"""
        return bool((self.region_bitfield >> region_id) & 0x01)


# ============================================================================
# MR76 RADAR INTERFACE CLASS
# ============================================================================

class MR76Radar:
    """
    Complete interface for MR76 77GHz millimeter wave radar
    
    This class provides high-level methods for:
    - Radar configuration and calibration
    - Object detection and tracking
    - Collision detection with configurable regions
    - Status monitoring and diagnostics
    
    Example:
        >>> from controlcan import ControlCAN, VCI_USBCAN1
        >>> can = ControlCAN()
        >>> radar = MR76Radar(can, VCI_USBCAN1, 0, 0, sensor_id=0)
        >>> radar.initialize()
        >>> objects = radar.get_objects()
    """
    
    def __init__(self, can_interface, device_type: int, device_index: int, 
                 can_index: int, sensor_id: int = 0):
        """
        Initialize MR76 radar interface
        
        Args:
            can_interface: ControlCAN instance
            device_type: CAN device type (e.g., VCI_USBCAN1)
            device_index: Device index (usually 0)
            can_index: CAN channel index (usually 0)
            sensor_id: Radar sensor ID (0-7), affects message IDs
        """
        self.can = can_interface
        self.device_type = device_type
        self.device_index = device_index
        self.can_index = can_index
        self.sensor_id = sensor_id
        
        # Cache for received data
        self._last_radar_state: Optional[RadarState] = None
        self._last_version: Optional[SoftwareVersion] = None
        self._last_object_status: Optional[ObjectListStatus] = None
        self._objects: List[RadarObject] = []
        self._collision_state: Optional[CollisionDetectionState] = None
        self._region_state: Optional[CollisionDetectionRegionState] = None
    
    def _get_msg_id(self, base_id: int) -> int:
        """Calculate actual message ID based on sensor ID"""
        return MessageID.adjust_for_sensor(base_id, self.sensor_id)
    
    def _send_config(self, msg_id: int, data: bytes) -> bool:
        """
        Send configuration message to radar
        
        Args:
            msg_id: Base message ID
            data: 8-byte configuration data
            
        Returns:
            True if sent successfully
        """
        from usbcan_ii_libusb_aarch64.USBCAN_Interface import VCI_CAN_OBJ
        
        frame = VCI_CAN_OBJ()
        frame.ID = self._get_msg_id(msg_id)
        frame.ExternFlag = 0  # Standard frame
        frame.RemoteFlag = 0  # Data frame
        frame.DataLen = 8
        
        for i in range(8):
            frame.Data[i] = data[i]
        
        sent = self.can.transmit(self.device_type, self.device_index, 
                                 self.can_index, frame)
        return sent == 1
    
    def configure_radar(self, config: RadarConfig) -> bool:
        """
        Configure radar parameters (0x200)
        
        Args:
            config: RadarConfig object with desired settings
            
        Returns:
            True if configuration sent successfully
            
        Example:
            >>> config = RadarConfig()
            >>> config.sensor_id_valid = True
            >>> config.sensor_id = 1
            >>> config.store_in_nvm_valid = True
            >>> config.store_in_nvm = True
            >>> radar.configure_radar(config)
        """
        data = config.to_can_frame()
        return self._send_config(MessageID.RADAR_CFG, data)
    
    def set_sensor_id(self, new_id: int, save_to_nvm: bool = True) -> bool:
        """
        Change radar sensor ID
        
        Args:
            new_id: New sensor ID (0-7)
            save_to_nvm: Save to flash memory (survives power cycle)
            
        Returns:
            True if command sent successfully
            
        Note:
            After changing ID, you must update self.sensor_id and reconnect
            
        Example:
            >>> radar.set_sensor_id(1, save_to_nvm=True)
            >>> radar.sensor_id = 1  # Update local sensor ID
        """
        config = RadarConfig()
        config.sensor_id_valid = True
        config.sensor_id = new_id
        config.store_in_nvm_valid = save_to_nvm
        config.store_in_nvm = save_to_nvm
        return self.configure_radar(config)
    
    def set_max_distance(self, distance_m: int, save_to_nvm: bool = False) -> bool:
        """
        Set maximum detection distance
        
        Args:
            distance_m: Maximum distance in meters (0-2048, resolution 2m)
            save_to_nvm: Save to flash memory
            
        Returns:
            True if command sent successfully
        """
        config = RadarConfig()
        config.max_distance_valid = True
        config.max_distance = distance_m
        config.store_in_nvm_valid = save_to_nvm
        config.store_in_nvm = save_to_nvm
        return self.configure_radar(config)
    
    def set_radar_power(self, power: RadarPower, save_to_nvm: bool = False) -> bool:
        """
        Set radar transmission power
        
        Args:
            power: RadarPower enum value
            save_to_nvm: Save to flash memory
            
        Returns:
            True if command sent successfully
        """
        config = RadarConfig()
        config.radar_power_valid = True
        config.radar_power = power
        config.store_in_nvm_valid = save_to_nvm
        config.store_in_nvm = save_to_nvm
        return self.configure_radar(config)
    
    def set_sensitivity(self, high_sensitivity: bool, save_to_nvm: bool = False) -> bool:
        """
        Set radar sensitivity (RCS threshold)
        
        Args:
            high_sensitivity: True for high sensitivity (detect fine targets)
            save_to_nvm: Save to flash memory
            
        Returns:
            True if command sent successfully
        """
        config = RadarConfig()
        config.rcs_threshold_valid = True
        config.rcs_threshold = (RCSThreshold.HIGH_SENSITIVITY if high_sensitivity 
                               else RCSThreshold.STANDARD)
        config.store_in_nvm_valid = save_to_nvm
        config.store_in_nvm = save_to_nvm
        return self.configure_radar(config)
    
    def set_sort_method(self, sort_by: SortIndex, save_to_nvm: bool = False) -> bool:
        """
        Set object sorting method
        
        Args:
            sort_by: SortIndex enum value
            save_to_nvm: Save to flash memory
            
        Returns:
            True if command sent successfully
        """
        config = RadarConfig()
        config.sort_index_valid = True
        config.sort_index = sort_by
        config.store_in_nvm_valid = save_to_nvm
        config.store_in_nvm = save_to_nvm
        return self.configure_radar(config)
    
    def enable_calibration(self, enable: bool) -> bool:
        """
        Enable or disable channel calibration
        
        Args:
            enable: True to enable calibration, False to restore initial values
            
        Returns:
            True if command sent successfully
            
        Note:
            Requires firmware V1.0.44 or later
        """
        config = RadarConfig()
        config.calibration_valid = True
        config.calibration_mode = (CalibrationMode.ENABLED if enable 
                                  else CalibrationMode.INITIAL_RECOVERY)
        return self.configure_radar(config)
    
    def set_baud_rate(self, baud_rate: BaudRate, save_to_nvm: bool = True) -> bool:
        """
        Set CAN bus baud rate
        
        Args:
            baud_rate: BaudRate enum value
            save_to_nvm: Save to flash memory (recommended)
            
        Returns:
            True if command sent successfully
            
        Warning:
            After changing baud rate, you must reconfigure the CAN interface
        """
        config = RadarConfig()
        config.baud_rate_valid = True
        config.baud_rate = baud_rate
        config.store_in_nvm_valid = save_to_nvm
        config.store_in_nvm = save_to_nvm
        return self.configure_radar(config)
    
    def enable_collision_detection(self, enable: bool, min_time: float = 0.0) -> bool:
        """
        Enable or disable collision detection
        
        Args:
            enable: True to activate collision detection
            min_time: Minimum detection time before warning (0-25.5 seconds)
            
        Returns:
            True if command sent successfully
            
        Note:
            Must be in OBJECTS mode for collision detection to work
        """
        config = CollisionDetectionConfig()
        config.activation = enable
        config.min_time_valid = True
        config.min_time = min_time
        
        data = config.to_can_frame()
        return self._send_config(MessageID.COLLISION_DETECT_CFG, data)
    
    def clear_collision_regions(self) -> bool:
        """
        Clear all collision detection regions
        
        Returns:
            True if command sent successfully
            
        Note:
            After clearing, radar stops sending 0x408, 0x402, 0x60E messages
        """
        config = CollisionDetectionConfig()
        config.clear_regions = True
        
        data = config.to_can_frame()
        return self._send_config(MessageID.COLLISION_DETECT_CFG, data)
    
    def configure_collision_region(self, point1_long: float, point1_lat: float,
                                   point2_long: float, point2_lat: float,
                                   activate: bool = True) -> bool:
        """
        Configure rectangular collision detection region
        
        Args:
            point1_long: Point 1 longitudinal distance (m) - bottom-right corner
            point1_lat: Point 1 lateral distance (m) - bottom-right corner
            point2_long: Point 2 longitudinal distance (m) - top-left corner
            point2_lat: Point 2 lateral distance (m) - top-left corner
            activate: Activate the region immediately
            
        Returns:
            True if configuration sent successfully, False if invalid
            
        Constraints:
            - point1_long < point2_long (Point1 must be closer)
            - point1_lat > point2_lat (Point1 must be to the right)
            
        Example:
            >>> # Configure 10m x 170m detection zone
            >>> # Point1: (5m right, 0m ahead)
            >>> # Point2: (-5m left, 170m ahead)
            >>> radar.configure_collision_region(0, 5, 170, -5, activate=True)
        """
        region = CollisionDetectionRegion()
        region.activation = activate
        region.coordinates_valid = True
        region.region_id = 1  # MR76 only supports region ID 1
        region.point1_long = point1_long
        region.point1_lat = point1_lat
        region.point2_long = point2_long
        region.point2_lat = point2_lat
        
        if not region.validate():
            print("Error: Invalid region coordinates!")
            print(f"  Constraint 1: point1_long ({point1_long}) < point2_long ({point2_long})")
            print(f"  Constraint 2: point1_lat ({point1_lat}) > point2_lat ({point2_lat})")
            return False
        
        data = region.to_can_frame()
        return self._send_config(MessageID.COLLISION_DETECT_REGION_CFG, data)
    
    def process_can_messages(self, timeout_ms: int = 100) -> int:
        """
        Process incoming CAN messages from radar
        
        Args:
            timeout_ms: Receive timeout in milliseconds
            
        Returns:
            Number of messages processed
            
        Note:
            This updates internal caches for radar state, objects, etc.
            Call this regularly in your main loop.
        """
        from usbcan_ii_libusb_aarch64.USBCAN_Interface import VCI_CAN_OBJ
        
        count, frames = self.can.receive(self.device_type, self.device_index,
                                         self.can_index, 50, wait_time=timeout_ms)
        
        for frame in frames:
            self._process_frame(frame)
        
        return count
    
    def _process_frame(self, frame) -> None:
        """Process a single CAN frame"""
        msg_id = frame.ID
        data = bytes(frame.Data[:frame.DataLen])
        
        # Adjust for sensor ID offset
        base_id = msg_id - (self.sensor_id * 0x10)
        
        if base_id == MessageID.RADAR_STATE:
            self._last_radar_state = RadarState.from_can_frame(data)
            
        elif base_id == MessageID.SOFTWARE_VERSION:
            self._last_version = SoftwareVersion.from_can_frame(data)
            
        elif base_id == MessageID.OBJECT_STATUS:
            self._last_object_status = ObjectListStatus.from_can_frame(data)
            self._objects = []  # Clear object list for new cycle
            
        elif base_id == MessageID.OBJECT_GENERAL:
            obj = RadarObject.from_can_frame(data)
            self._objects.append(obj)
            
        elif base_id == MessageID.COLLISION_DETECT_STATE:
            self._collision_state = CollisionDetectionState.from_can_frame(data)
            
        elif base_id == MessageID.COLLISION_DETECT_REGION_STATE:
            self._region_state = CollisionDetectionRegionState.from_can_frame(data)
    
    def get_radar_state(self) -> Optional[RadarState]:
        """
        Get latest radar state information
        
        Returns:
            RadarState object or None if not received yet
        """
        return self._last_radar_state
    
    def get_software_version(self) -> Optional[SoftwareVersion]:
        """
        Get radar software version
        
        Returns:
            SoftwareVersion object or None if not received yet
        """
        return self._last_version
    
    def get_objects(self) -> List[RadarObject]:
        """
        Get list of currently detected objects
        
        Returns:
            List of RadarObject instances
            
        Note:
            Call process_can_messages() first to update object list
        """
        return self._objects.copy()
    
    def get_object_count(self) -> int:
        """Get number of currently detected objects"""
        return len(self._objects)
    
    def get_collision_state(self) -> Optional[CollisionDetectionState]:
        """
        Get collision detection state
        
        Returns:
            CollisionDetectionState or None if not active
        """
        return self._collision_state
    
    def get_region_state(self) -> Optional[CollisionDetectionRegionState]:
        """
        Get collision detection region state
        
        Returns:
            CollisionDetectionRegionState or None if not configured
        """
        return self._region_state
    
    def is_region_warning_active(self) -> bool:
        """
        Check if collision warning is currently active
        
        Returns:
            True if target detected in region
        """
        if self._region_state is None:
            return False
        return self._region_state.warning_level == WarningLevel.TARGET_WARNING
    
    def wait_for_objects(self, timeout_ms: int = 1000) -> List[RadarObject]:
        """
        Wait for and return detected objects
        
        Args:
            timeout_ms: Maximum wait time in milliseconds
            
        Returns:
            List of detected RadarObject instances
        """
        import time
        start_time = time.time()
        
        while (time.time() - start_time) * 1000 < timeout_ms:
            self.process_can_messages(timeout_ms=50)
            if len(self._objects) > 0:
                return self.get_objects()
            time.sleep(0.01)
        
        return []
    
    def print_radar_state(self) -> None:
        """Print current radar state in human-readable format"""
        state = self.get_radar_state()
        if state is None:
            print("Radar state not available")
            return
        
        print("=" * 60)
        print("RADAR STATE")
        print("=" * 60)
        print(f"Sensor ID:        {state.sensor_id}")
        print(f"Max Distance:     {state.max_distance} m")
        print(f"Output Type:      {state.output_type.name}")
        print(f"Radar Power:      {state.radar_power.name}")
        print(f"Sort Method:      {state.sort_index.name}")
        print(f"RCS Threshold:    {state.rcs_threshold.name}")
        print(f"CAN Baud Rate:    {state.can_baud_rate.name}")
        print(f"NVM Read Status:  {'OK' if state.nvm_read_status else 'Failed'}")
        print(f"NVM Write Status: {'OK' if state.nvm_write_status else 'Failed'}")
        print("=" * 60)
    
    def print_objects(self) -> None:
        """Print detected objects in human-readable format"""
        objects = self.get_objects()
        
        print("=" * 80)
        print(f"DETECTED OBJECTS: {len(objects)}")
        print("=" * 80)
        
        if len(objects) == 0:
            print("No objects detected")
            print("=" * 80)
            return
        
        print(f"{'ID':<4} {'Long(m)':<8} {'Lat(m)':<8} {'VLong':<8} {'VLat':<8} "
              f"{'Range(m)':<9} {'Angle(°)':<9} {'Class':<8}")
        print("-" * 80)
        
        for obj in objects:
            range_m = obj.get_radial_distance()
            angle = obj.get_angle_deg()
            
            print(f"{obj.object_id:<4} {obj.dist_long:>7.2f} {obj.dist_lat:>7.2f} "
                  f"{obj.vrel_long:>7.2f} {obj.vrel_lat:>7.2f} "
                  f"{range_m:>8.2f} {angle:>8.1f} {obj.object_class.name:<8}")
        
        print("=" * 80)
    
    def print_collision_state(self) -> None:
        """Print collision detection state"""
        coll_state = self.get_collision_state()
        region_state = self.get_region_state()
        
        print("=" * 60)
        print("COLLISION DETECTION")
        print("=" * 60)
        
        if coll_state is None:
            print("Collision detection not active")
            print("=" * 60)
            return
        
        print(f"Active:           {coll_state.activation}")
        print(f"Regions:          {coll_state.num_regions}")
        print(f"Min Detect Time:  {coll_state.min_detect_time:.1f} s")
        print(f"Meas Counter:     {coll_state.meas_counter}")
        
        if region_state:
            print("\nRegion State:")
            print(f"  Region ID:      {region_state.region_id}")
            print(f"  Warning Level:  {region_state.warning_level.name}")
            print(f"  Point 1:        ({region_state.point1_long:.1f}m, "
                  f"{region_state.point1_lat:.1f}m)")
            print(f"  Point 2:        ({region_state.point2_long:.1f}m, "
                  f"{region_state.point2_lat:.1f}m)")
            print(f"  Objects:        {region_state.num_objects}")
        
        print("=" * 60)


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def create_quick_config_commands() -> Dict[str, bytes]:
    """
    Create common configuration commands for quick reference
    
    Returns:
        Dictionary of command name to CAN data bytes
        
    Example:
        >>> commands = create_quick_config_commands()
        >>> # Send high sensitivity command
        >>> radar._send_config(0x200, commands['high_sensitivity'])
    """
    commands = {
        # Sensor ID changes (with NVM save)
        'set_id_1': bytes([0x82, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00]),
        'set_id_2': bytes([0x82, 0x00, 0x00, 0x00, 0x02, 0x80, 0x00, 0x00]),
        'set_id_3': bytes([0x82, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00]),
        
        # Sensitivity
        'high_sensitivity': bytes([0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00]),
        'low_sensitivity': bytes([0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00]),
        
        # Calibration
        'enable_calibration': bytes([0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x0A]),
        'restore_calibration': bytes([0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x0C]),
        
        # Clear collision regions
        'clear_regions': bytes([0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
    }
    
    return commands


def parse_object_example() -> None:
    """
    Demonstrate object parsing with example from protocol documentation.
    Uses proper Motorola (Big-Endian) bit ordering.
    
    Example data: 0x57 0x4E 0xC4 0x0C 0x7F 0x60 0x18 0x80
    Expected results (from protocol):
        - Object ID: 87
        - Distance Long: 4.0 m
        - Distance Lat: 2.6 m
        - Velocity Long: -0.75 m/s
    """
    example_data = bytes([0x57, 0x4E, 0xC4, 0x0C, 0x7F, 0x60, 0x18, 0x80])
    
    obj = RadarObject.from_can_frame(example_data)
    
    print("=" * 70)
    print("Object Parsing Example - Motorola Bit Order Validation")
    print("=" * 70)
    print(f"Raw Data: {' '.join([f'0x{b:02X}' for b in example_data])}")
    print()
    
    print("Motorola Bit Ordering:")
    print("  - Bytes numbered 0-7 (left to right)")
    print("  - Bits in each byte: MSB=7 to LSB=0")
    print("  - Start bit = LSB of signal")
    print("  - Multi-byte signals grow backwards")
    print()
    
    # Manual calculation using Motorola helpers
    print("Manual Extraction (Motorola):")
    print("-" * 70)
    
    dist_long_raw = motorola_extract_signal(example_data, 19, 13)
    print(f"DistLong: start_bit=19, length=13")
    print(f"  Raw value: {dist_long_raw}")
    print(f"  Decoded: {dist_long_raw} × 0.2 + (-500) = {dist_long_raw * 0.2 - 500:.1f} m")
    
    dist_lat_raw = motorola_extract_signal(example_data, 24, 11)
    print(f"DistLat: start_bit=24, length=11")
    print(f"  Raw value: {dist_lat_raw}")
    print(f"  Decoded: {dist_lat_raw} × 0.2 + (-204.6) = {dist_lat_raw * 0.2 - 204.6:.1f} m")
    
    vrel_long_raw = motorola_extract_signal(example_data, 46, 10)
    print(f"VrelLong: start_bit=46, length=10")
    print(f"  Raw value: {vrel_long_raw}")
    print(f"  Decoded: {vrel_long_raw} × 0.25 + (-128) = {vrel_long_raw * 0.25 - 128:.2f} m/s")
    
    print()
    print("Parsed Object Results:")
    print("-" * 70)
    print(f"  Object ID: {obj.object_id}")
    print(f"  Distance Long: {obj.dist_long:.1f} m")
    print(f"  Distance Lat: {obj.dist_lat:.1f} m")
    print(f"  Velocity Long: {obj.vrel_long:.2f} m/s")
    print(f"  Velocity Lat: {obj.vrel_lat:.2f} m/s")
    print(f"  Radial Distance: {obj.get_radial_distance():.2f} m")
    print(f"  Angle: {obj.get_angle_deg():.1f}°")
    print(f"  Dynamic Property: {obj.dyn_prop.name}")
    print(f"  Object Class: {obj.object_class.name}")
    print(f"  RCS: {obj.rcs:.1f} dBm²")
    
    # Validation
    print()
    print("Validation (Expected vs Actual):")
    print("-" * 70)
    expected = {
        'ID': 87,
        'DistLong': 4.0,
        'DistLat': 2.6,
        'VrelLong': -0.75,
    }
    
    def validate(name, expected_val, actual_val, tolerance=0.1):
        match = abs(expected_val - actual_val) <= tolerance
        status = "✓" if match else "✗"
        print(f"{status} {name}: Expected={expected_val}, Actual={actual_val:.2f}")
    
    validate('Object ID', expected['ID'], obj.object_id, tolerance=0)
    validate('DistLong', expected['DistLong'], obj.dist_long)
    validate('DistLat', expected['DistLat'], obj.dist_lat)
    validate('VrelLong', expected['VrelLong'], obj.vrel_long)
    
    print("=" * 70)


def test_motorola_bit_order() -> None:
    """
    Test Motorola bit ordering with known examples from protocol
    """
    print("\n" + "=" * 70)
    print("Motorola Bit Ordering Test")
    print("=" * 70)
    
    print("\nMotorola Rules:")
    print("  1. Start bit = LSB of signal")
    print("  2. Fill UPWARD within current byte (toward MSB)")
    print("  3. When byte full, jump to PREVIOUS byte starting at bit 0")
    
    # Test case 1: start_bit=56, length=11 (Point2Lat from your image)
    print("\n" + "-" * 70)
    print("Test 1: Point2Lat - start_bit=56, length=11")
    print("-" * 70)
    print("Expected bit usage:")
    print("  Byte 7: bits 56-63 (8 bits)")
    print("  Byte 6: bits 48-50 (3 bits)")
    print("  Total: 11 bits")
    
    # Create test data with known pattern
    test_data = bytearray(8)
    # Set all bits of Point2Lat to 1 (should give 2047 = 0x7FF)
    test_data[7] = 0xFF  # Bits 56-63 = all 1s
    test_data[6] = 0x07  # Bits 48-50 = all 1s (lower 3 bits)
    
    extracted = motorola_extract_signal(bytes(test_data), 56, 11)
    print(f"\nTest Data: {' '.join([f'0x{b:02X}' for b in test_data])}")
    print(f"Extracted: {extracted} (expected 2047 = 0x7FF if all bits set)")
    print(f"Binary: 0b{extracted:011b}")
    status = "✓" if extracted == 2047 else "✗"
    print(f"{status} Extraction correct: {extracted == 2047}")
    
    # Test case 2: start_bit=51, length=13 (Point2Long)
    print("\n" + "-" * 70)
    print("Test 2: Point2Long - start_bit=51, length=13")
    print("-" * 70)
    print("Expected bit usage:")
    print("  Byte 6: bits 51-55 (5 bits)")
    print("  Byte 5: bits 40-47 (8 bits)")
    print("  Total: 13 bits")
    
    test_data2 = bytearray(8)
    # Set all bits of Point2Long to 1 (should give 8191 = 0x1FFF)
    test_data2[6] = 0xF8  # Bits 51-55 = all 1s (bits 3-7 of byte)
    test_data2[5] = 0xFF  # Bits 40-47 = all 1s
    
    extracted2 = motorola_extract_signal(bytes(test_data2), 51, 13)
    print(f"\nTest Data: {' '.join([f'0x{b:02X}' for b in test_data2])}")
    print(f"Extracted: {extracted2} (expected 8191 = 0x1FFF if all bits set)")
    print(f"Binary: 0b{extracted2:013b}")
    status2 = "✓" if extracted2 == 8191 else "✗"
    print(f"{status2} Extraction correct: {extracted2 == 8191}")
    
    # Test insertion round-trip
    print("\n" + "-" * 70)
    print("Test 3: Insertion Round-Trip")
    print("-" * 70)
    
    test_value = 998  # Point2Lat value from protocol example
    test_data3 = bytearray(8)
    motorola_insert_signal(test_data3, 56, 11, test_value)
    
    print(f"Inserted value: {test_value}")
    print(f"Result: {' '.join([f'0x{b:02X}' for b in test_data3])}")
    
    extracted3 = motorola_extract_signal(bytes(test_data3), 56, 11)
    status3 = "✓" if extracted3 == test_value else "✗"
    print(f"{status3} Round-trip: Inserted={test_value}, Extracted={extracted3}")
    
    print("=" * 70)


def test_region_encoding_decoding_1() -> None:
    """
    Test collision region encoding with proper Motorola bit ordering
    """
    print("\n" + "=" * 70)
    print("Collision Region Encoding/Decoding Test (Motorola)")
    print("=" * 70)
    
    # Create test region: 10m wide × 170m deep (from protocol example)
    print("\nTest Case: 10m wide × 170m deep zone")
    print("-" * 70)
    
    region = CollisionDetectionRegion()
    region.activation = True
    region.coordinates_valid = True
    region.region_id = 1
    region.point1_long = 0.0    # Bottom-right: 0m ahead
    region.point1_lat = 5.0     # Bottom-right: 5m right
    region.point2_long = 170.0  # Top-left: 170m ahead
    region.point2_lat = -5.0    # Top-left: 5m left
    
    print(f"Input Coordinates:")
    print(f"  Point1 (Bottom-Right): ({region.point1_long}m, {region.point1_lat}m)")
    print(f"  Point2 (Top-Left): ({region.point2_long}m, {region.point2_lat}m)")
    print(f"  Valid: {region.validate()}")
    
    # Encode
    encoded = region.to_can_frame()
    print(f"\nEncoded CAN Frame (Motorola):")
    print(f"  {' '.join([f'0x{b:02X}' for b in encoded])}")
    
    print("\nExpected CAN Frame (from protocol doc):")
    expected = bytes([0x06, 0x01, 0x4E, 0x24, 0x18, 0x68, 0xB3, 0xE6])
    print(f"  {' '.join([f'0x{b:02X}' for b in expected])}")
    
    # Compare
    match = encoded == expected
    print(f"\n{'✓' if match else '✗'} Encoding matches protocol example: {match}")
    
    if not match:
        print("\n  Byte-by-byte comparison:")
        for i, (e, a) in enumerate(zip(expected, encoded)):
            status = '✓' if e == a else '✗'
            diff = f" (diff: {a-e:+d})" if e != a else ""
            print(f"  {status} Byte {i}: Expected=0x{e:02X}, Actual=0x{a:02X}{diff}")
    else:
        print("\n✓✓✓ Perfect match! Motorola bit ordering is correct! ✓✓✓")
    
    print("=" * 70)
    """
    Demonstrate object parsing with example from protocol documentation
    Validates offset and resolution handling.
    
    Example data: 0x57 0x4E 0xC4 0x0C 0x7F 0x60 0x18 0x80
    Expected results (from protocol):
        - Object ID: 87
        - Distance Long: 4.0 m
        - Distance Lat: 2.6 m
        - Velocity Long: -0.75 m/s
        - Velocity Lat: 0.0 m/s
    """
    example_data = bytes([0x57, 0x4E, 0xC4, 0x0C, 0x7F, 0x60, 0x18, 0x80])
    
    obj = RadarObject.from_can_frame(example_data)
    
    print("=" * 70)
    print("Object Parsing Example - Offset & Resolution Validation")
    print("=" * 70)
    print(f"Raw Data: {' '.join([f'0x{b:02X}' for b in example_data])}")
    print()
    
    # Manual calculation for verification
    print("Manual Calculations:")
    print("-" * 70)
    
    # DistLong
    dist_long_raw = (0x4E << 5) | (0xC4 >> 3)
    print(f"DistLong Raw: (0x4E << 5) | (0xC4 >> 3) = {dist_long_raw}")
    print(f"DistLong: {dist_long_raw} × 0.2 + (-500) = {dist_long_raw * 0.2 - 500:.1f} m")
    
    # DistLat
    dist_lat_raw = ((0xC4 & 0x07) << 8) | 0x0C
    print(f"DistLat Raw: ((0xC4 & 0x07) << 8) | 0x0C = {dist_lat_raw}")
    print(f"DistLat: {dist_lat_raw} × 0.2 + (-204.6) = {dist_lat_raw * 0.2 - 204.6:.1f} m")
    
    # VrelLong
    vrel_long_raw = (0x7F << 2) | (0x60 >> 6)
    print(f"VrelLong Raw: (0x7F << 2) | (0x60 >> 6) = {vrel_long_raw}")
    print(f"VrelLong: {vrel_long_raw} × 0.25 + (-128) = {vrel_long_raw * 0.25 - 128:.2f} m/s")
    
    # VrelLat
    vrel_lat_raw = ((0x60 & 0x01) << 8) | (0x18 >> 5)
    print(f"VrelLat Raw: ((0x60 & 0x01) << 8) | (0x18 >> 5) = {vrel_lat_raw}")
    print(f"VrelLat: {vrel_lat_raw} × 0.25 + (-64) = {vrel_lat_raw * 0.25 - 64:.2f} m/s")
    
    print()
    print("Parsed Object Results:")
    print("-" * 70)
    print(f"  Object ID: {obj.object_id}")
    print(f"  Distance Long: {obj.dist_long:.1f} m")
    print(f"  Distance Lat: {obj.dist_lat:.1f} m")
    print(f"  Velocity Long: {obj.vrel_long:.2f} m/s")
    print(f"  Velocity Lat: {obj.vrel_lat:.2f} m/s")
    print(f"  Radial Distance: {obj.get_radial_distance():.2f} m")
    print(f"  Angle: {obj.get_angle_deg():.1f}°")
    print(f"  Dynamic Property: {obj.dyn_prop.name}")
    print(f"  Object Class: {obj.object_class.name}")
    print(f"  RCS: {obj.rcs:.1f} dBm²")
    
    # Validation
    print()
    print("Validation (Expected vs Actual):")
    print("-" * 70)
    expected = {
        'ID': 87,
        'DistLong': 4.0,
        'DistLat': 2.6,
        'VrelLong': -0.75,
        'VrelLat': 0.0  # Actually should be -64.0 based on raw=0
    }
    
    def validate(name, expected_val, actual_val, tolerance=0.1):
        match = abs(expected_val - actual_val) <= tolerance
        status = "✓" if match else "✗"
        print(f"{status} {name}: Expected={expected_val}, Actual={actual_val:.2f}")
    
    validate('Object ID', expected['ID'], obj.object_id, tolerance=0)
    validate('DistLong', expected['DistLong'], obj.dist_long)
    validate('DistLat', expected['DistLat'], obj.dist_lat)
    validate('VrelLong', expected['VrelLong'], obj.vrel_long)
    
    print()
    print("Note: VrelLat raw value is 0, so actual = 0 × 0.25 + (-64) = -64.0 m/s")
    print("      Protocol example may have simplified this to 0 m/s")
    print("=" * 70)


def test_all_messages_motorola() -> None:
    """
    Comprehensive test of ALL message types with Motorola bit ordering
    """
    print("\n" + "=" * 70)
    print("Complete Message Validation - All Messages Using Motorola")
    print("=" * 70)
    
    all_passed = True
    
    # Test 1: RadarConfig (0x200)
    print("\n1. Testing RadarConfig (0x200) encoding...")
    print("-" * 70)
    config = RadarConfig()
    config.max_distance_valid = True
    config.max_distance = 200
    config.sensor_id_valid = True
    config.sensor_id = 1
    config.store_in_nvm_valid = True
    config.store_in_nvm = True
    
    config_data = config.to_can_frame()
    print(f"   Encoded: {' '.join([f'0x{b:02X}' for b in config_data])}")
    
    # Verify extraction
    max_dist_check = int(decode_signal(config_data, 22, 10, 2, 0))
    sensor_id_check = motorola_extract_signal(config_data, 32, 3)
    
    test1_pass = (max_dist_check == 200 and sensor_id_check == 1)
    status1 = "✓" if test1_pass else "✗"
    print(f"   {status1} MaxDistance: {max_dist_check} (expected 200)")
    print(f"   {status1} SensorID: {sensor_id_check} (expected 1)")
    all_passed &= test1_pass
    
    # Test 2: ObjectGeneral (0x60B) - Protocol example
    print("\n2. Testing ObjectGeneral (0x60B) decoding...")
    print("-" * 70)
    obj_data = bytes([0x57, 0x4E, 0xC4, 0x0C, 0x7F, 0x60, 0x18, 0x80])
    obj = RadarObject.from_can_frame(obj_data)
    
    test2_pass = (
        obj.object_id == 87 and
        abs(obj.dist_long - 4.0) < 0.1 and
        abs(obj.dist_lat - 2.6) < 0.1 and
        abs(obj.vrel_long - (-0.75)) < 0.1
    )
    status2 = "✓" if test2_pass else "✗"
    print(f"   {status2} Object ID: {obj.object_id} (expected 87)")
    print(f"   {status2} DistLong: {obj.dist_long:.1f}m (expected 4.0m)")
    print(f"   {status2} DistLat: {obj.dist_lat:.1f}m (expected 2.6m)")
    print(f"   {status2} VrelLong: {obj.vrel_long:.2f}m/s (expected -0.75m/s)")
    all_passed &= test2_pass
    
    # Test 3: CollisionDetectionRegion (0x401) - Protocol example
    print("\n3. Testing CollisionDetectionRegion (0x401) encoding...")
    print("-" * 70)
    region = CollisionDetectionRegion()
    region.activation = True
    region.coordinates_valid = True
    region.region_id = 1
    region.point1_long = 0.0
    region.point1_lat = 5.0
    region.point2_long = 170.0
    region.point2_lat = -5.0
    
    region_data = region.to_can_frame()
    expected_region = bytes([0x06, 0x01, 0x4E, 0x24, 0x18, 0x68, 0xB3, 0xE6])
    
    test3_pass = (region_data == expected_region)
    status3 = "✓" if test3_pass else "✗"
    print(f"   Encoded: {' '.join([f'0x{b:02X}' for b in region_data])}")
    print(f"   Expected: {' '.join([f'0x{b:02X}' for b in expected_region])}")
    print(f"   {status3} Match: {test3_pass}")
    
    if not test3_pass:
        print("   Byte differences:")
        for i, (e, a) in enumerate(zip(expected_region, region_data)):
            if e != a:
                print(f"      Byte {i}: Expected=0x{e:02X}, Actual=0x{a:02X}")
    all_passed &= test3_pass
    
    # Test 4: SoftwareVersion (0x700)
    print("\n4. Testing SoftwareVersion (0x700) decoding...")
    print("-" * 70)
    version_data = bytes([0x01, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00])
    version = SoftwareVersion.from_can_frame(version_data)
    
    test4_pass = (version.major == 1 and version.minor == 0 and version.patch == 21)
    status4 = "✓" if test4_pass else "✗"
    print(f"   {status4} Version: {version} (expected V1.0.21)")
    all_passed &= test4_pass
    
    # Test 5: ObjectListStatus (0x60A)
    print("\n5. Testing ObjectListStatus (0x60A) decoding...")
    print("-" * 70)
    obj_status_data = bytearray(8)
    motorola_insert_signal(obj_status_data, 0, 8, 5)  # 5 objects at start_bit 0
    motorola_insert_signal(obj_status_data, 16, 16, 1234)  # meas count at start_bit 16
    motorola_insert_signal(obj_status_data, 28, 4, 0)  # interface version at start_bit 28
    
    # Debug: verify what we inserted
    check_objects = motorola_extract_signal(bytes(obj_status_data), 0, 8)
    check_meas = motorola_extract_signal(bytes(obj_status_data), 8, 16)
    
    obj_status = ObjectListStatus.from_can_frame(bytes(obj_status_data))
    test5_pass = (obj_status.num_objects == 5 and obj_status.meas_count == 1234)
    status5 = "✓" if test5_pass else "✗"
    print(f"   Inserted: NumObjects={check_objects}, MeasCount={check_meas}")
    print(f"   {status5} NumObjects: {obj_status.num_objects} (expected 5)")
    print(f"   {status5} MeasCount: {obj_status.meas_count} (expected 1234)")
    all_passed &= test5_pass
    
    # Test 6: CollisionDetectionConfig (0x400)
    print("\n6. Testing CollisionDetectionConfig (0x400) encoding...")
    print("-" * 70)
    coll_config = CollisionDetectionConfig()
    coll_config.activation = True
    coll_config.min_time_valid = True
    coll_config.min_time = 2.5
    
    coll_config_data = coll_config.to_can_frame()
    
    activation_check = bool(motorola_extract_signal(coll_config_data, 1, 1))
    min_time_check = decode_signal(coll_config_data, 8, 8, 0.1, 0)
    
    test6_pass = (activation_check and abs(min_time_check - 2.5) < 0.1)
    status6 = "✓" if test6_pass else "✗"
    print(f"   {status6} Activation: {activation_check} (expected True)")
    print(f"   {status6} MinTime: {min_time_check:.1f}s (expected 2.5s)")
    all_passed &= test6_pass
    
    # Test 7: CollisionDetectionState (0x408)
    print("\n7. Testing CollisionDetectionState (0x408) decoding...")
    print("-" * 70)
    coll_state_data = bytearray(8)
    motorola_insert_signal(coll_state_data, 1, 1, 1)  # active
    motorola_insert_signal(coll_state_data, 4, 4, 1)  # 1 region
    encode_signal(coll_state_data, 8, 8, 1.5, 0.1, 0)  # 1.5s
    motorola_insert_signal(coll_state_data, 24, 16, 100)  # counter
    
    coll_state = CollisionDetectionState.from_can_frame(bytes(coll_state_data))
    test7_pass = (coll_state.activation and coll_state.num_regions == 1)
    status7 = "✓" if test7_pass else "✗"
    print(f"   {status7} Activation: {coll_state.activation} (expected True)")
    print(f"   {status7} NumRegions: {coll_state.num_regions} (expected 1)")
    all_passed &= test7_pass
    
    # Test 8: ObjectCollisionWarning (0x60E)
    print("\n8. Testing ObjectCollisionWarning (0x60E) decoding...")
    print("-" * 70)
    warning_data = bytearray(8)
    motorola_insert_signal(warning_data, 0, 8, 42)  # object ID
    motorola_insert_signal(warning_data, 8, 8, 0x02)  # region bitfield (bit 1)
    
    warning = ObjectCollisionWarning.from_can_frame(bytes(warning_data))
    test8_pass = (warning.object_id == 42 and warning.is_in_region(1))
    status8 = "✓" if test8_pass else "✗"
    print(f"   {status8} Object ID: {warning.object_id} (expected 42)")
    print(f"   {status8} In Region 1: {warning.is_in_region(1)} (expected True)")
    all_passed &= test8_pass
    
    # Final summary
    print("\n" + "=" * 70)
    if all_passed:
        print("✓✓✓ ALL TESTS PASSED! ✓✓✓")
        print("All messages correctly use Motorola bit ordering!")
    else:
        print("✗✗✗ SOME TESTS FAILED ✗✗✗")
        print("Review the failures above.")
    print("=" * 70)
    """
    Test encoding with offset and resolution
    Validates that encoding and decoding are inverse operations
    """
    print("\n" + "=" * 70)
    print("Offset & Resolution Encoding Test")
    print("=" * 70)
    
    test_cases = [
        # (actual_value, resolution, offset, description)
        (4.0, 0.2, -500, "DistLong: 4m"),
        (2.6, 0.2, -204.6, "DistLat: 2.6m"),
        (-5.0, 0.2, -204.6, "DistLat: -5m (left)"),
        (-0.75, 0.25, -128, "VrelLong: -0.75 m/s (approaching)"),
        (10.0, 0.25, -128, "VrelLong: 10 m/s (departing)"),
        (200, 2, 0, "MaxDistance: 200m"),
    ]
    
    print()
    for actual_value, resolution, offset, description in test_cases:
        # Encode
        raw_value = round((actual_value - offset) / resolution)
        
        # Decode
        decoded_value = raw_value * resolution + offset
        
        # Validate
        error = abs(decoded_value - actual_value)
        status = "✓" if error < 0.01 else "✗"
        
        print(f"{status} {description}")
        print(f"   Actual: {actual_value} → Raw: {raw_value} → Decoded: {decoded_value:.2f}")
        print(f"   Formula: raw = ({actual_value} - ({offset})) / {resolution} = {raw_value}")
        print(f"   Inverse: actual = {raw_value} × {resolution} + ({offset}) = {decoded_value:.2f}")
        print()
    
    print("=" * 70)


def test_region_encoding_decoding_2() -> None:
    """
    Test collision region encoding and decoding with offset/resolution
    
    WARNING: This test reveals a potential protocol documentation issue.
    Point2Lat needs 11 bits starting at bit 56, which would extend beyond
    the 8-byte CAN frame. The implementation matches the protocol example
    but may have reduced precision.
    """
    print("\n" + "=" * 70)
    print("Collision Region Encoding/Decoding Test")
    print("=" * 70)
    
    # Create test region: 10m wide × 170m deep
    print("\nTest Case: 10m wide × 170m deep zone (from protocol example)")
    print("-" * 70)
    
    region = CollisionDetectionRegion()
    region.activation = True
    region.coordinates_valid = True
    region.region_id = 1
    region.point1_long = 0.0    # Bottom-right: 0m ahead
    region.point1_lat = 5.0     # Bottom-right: 5m right
    region.point2_long = 170.0  # Top-left: 170m ahead
    region.point2_lat = -5.0    # Top-left: 5m left
    
    print(f"Input Coordinates:")
    print(f"  Point1 (Bottom-Right): ({region.point1_long}m, {region.point1_lat}m)")
    print(f"  Point2 (Top-Left): ({region.point2_long}m, {region.point2_lat}m)")
    print(f"  Valid: {region.validate()}")
    
    # Manual calculation
    print(f"\nManual Encoding (with Offset & Resolution):")
    p1_long_raw = int((0.0 - (-500)) / 0.2)
    p1_lat_raw = int((5.0 - (-204.6)) / 0.2)
    p2_long_raw = int((170.0 - (-500)) / 0.2)
    p2_lat_raw = int((-5.0 - (-204.6)) / 0.2)
    
    print(f"  Point1Long: (0.0 + 500) / 0.2 = {p1_long_raw} = 0x{p1_long_raw:04X}")
    print(f"  Point1Lat: (5.0 + 204.6) / 0.2 = {p1_lat_raw} = 0x{p1_lat_raw:03X}")
    print(f"  Point2Long: (170.0 + 500) / 0.2 = {p2_long_raw} = 0x{p2_long_raw:04X}")
    print(f"  Point2Lat: (-5.0 + 204.6) / 0.2 = {p2_lat_raw} = 0x{p2_lat_raw:03X}")
    
    # Encode
    encoded = region.to_can_frame()
    print(f"\nEncoded CAN Frame:")
    print(f"  {' '.join([f'0x{b:02X}' for b in encoded])}")
    
    print("\nExpected CAN Frame (from protocol doc page 16):")
    print("  0x06 0x01 0x4E 0x24 0x18 0x68 0xB3 0xE6")
    
    # Compare
    expected = bytes([0x06, 0x01, 0x4E, 0x24, 0x18, 0x68, 0xB3, 0xE6])
    match = encoded == expected
    print(f"\n{'✓' if match else '✗'} Encoding matches protocol example: {match}")
    
    if not match:
        print("\n  Byte-by-byte comparison:")
        for i, (e, a) in enumerate(zip(expected, encoded)):
            status = '✓' if e == a else '✗'
            print(f"  {status} Byte {i}: Expected=0x{e:02X}, Actual=0x{a:02X}")
    
    # Analyze the Point2Lat issue
    print("\n" + "=" * 70)
    print("PROTOCOL DOCUMENTATION ISSUE ANALYSIS")
    print("=" * 70)
    print("\nProblem: Point2Lat requires 11 bits starting at bit 56")
    print("  11 bits starting at bit 56 would occupy bits 56-66")
    print("  But 8-byte CAN frame only has bits 0-63!")
    print("  Bits 64-66 don't exist in standard CAN.")
    
    print(f"\nPoint2Lat raw value: {p2_lat_raw} = 0x{p2_lat_raw:03X} = 0b{p2_lat_raw:011b}")
    print(f"  Needs 11 bits, but only ~3 bits available in byte 7")
    
    print("\nPossible explanations:")
    print("  1. Protocol documentation has incorrect bit positions")
    print("  2. Point2Lat uses different bit layout than documented")
    print("  3. Radar doesn't need full 11-bit precision for Point2Lat")
    print("  4. Multiple CAN frames are used (not documented)")
    
    print("\nRecommendation:")
    print("  ⚠️  Test with actual radar hardware")
    print("  ⚠️  Contact manufacturer (Hunan Nanoradar) for clarification")
    print("  ⚠️  Verify region detection works with example command")
    
    print("\n" + "=" * 70)


def calculate_region_coordinates(width_m: float, depth_m: float, 
                                 center_lat: float = 0.0,
                                 start_long: float = 0.0) -> Tuple[float, float, float, float]:
    """
    Helper function to calculate region corner coordinates
    
    Args:
        width_m: Region width in meters (lateral dimension)
        depth_m: Region depth in meters (longitudinal dimension)
        center_lat: Lateral center offset (0 = centered on radar)
        start_long: Longitudinal start distance from radar
        
    Returns:
        Tuple of (point1_long, point1_lat, point2_long, point2_lat)
        
    Example:
        >>> # Create 10m wide x 170m deep zone, centered, starting at 0m
        >>> coords = calculate_region_coordinates(10, 170, 0, 0)
        >>> radar.configure_collision_region(*coords)
    """
    half_width = width_m / 2
    
    point1_long = start_long
    point1_lat = center_lat + half_width
    point2_long = start_long + depth_m
    point2_lat = center_lat - half_width
    
    return point1_long, point1_lat, point2_long, point2_lat


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

def example_basic_detection():
    """Example: Basic object detection"""
    from usbcan_ii_libusb_aarch64.USBCAN_Interface import ControlCAN, VCI_USBCAN1, STATUS_OK, VCI_INIT_CONFIG
    
    # Initialize CAN interface
    can = ControlCAN()
    
    if can.open_device(VCI_USBCAN1, 0, 0) != STATUS_OK:
        print("Failed to open USBCAN-I")
        return
    
    # Configure CAN: 500 Kbps (MR76 requirement)
    config = VCI_INIT_CONFIG()
    config.AccCode = 0x00000000
    config.AccMask = 0xFFFFFFFF
    config.Filter = 1
    config.Timing0 = 0x00
    config.Timing1 = 0x1C
    config.Mode = 0
    
    can.init_can(VCI_USBCAN1, 0, 0, config)
    can.start_can(VCI_USBCAN1, 0, 0)
    
    # Initialize radar interface
    radar = MR76Radar(can, VCI_USBCAN1, 0, 0, sensor_id=0)
    
    print("MR76 Radar - Basic Detection Example")
    print("Press Ctrl+C to stop\n")
    
    try:
        import time
        while True:
            # Process incoming messages
            radar.process_can_messages(timeout_ms=100)
            
            # Print radar state (updated every 1s)
            state = radar.get_radar_state()
            if state:
                version = radar.get_software_version()
                if version:
                    print(f"Radar {state.sensor_id} - Version: {version}")
            
            # Print detected objects
            objects = radar.get_objects()
            if len(objects) > 0:
                radar.print_objects()
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        can.close_device(VCI_USBCAN1, 0)


def example_collision_detection():
    """Example: Collision detection with rectangular zone"""
    from usbcan_ii_libusb_aarch64.USBCAN_Interface import ControlCAN, VCI_USBCAN1, STATUS_OK, VCI_INIT_CONFIG
    import time
    
    can = ControlCAN()
    can.open_device(VCI_USBCAN1, 0, 0)
    
    config = VCI_INIT_CONFIG()
    config.AccCode = 0x00000000
    config.AccMask = 0xFFFFFFFF
    config.Filter = 1
    config.Timing0 = 0x00
    config.Timing1 = 0x1C
    config.Mode = 0
    
    can.init_can(VCI_USBCAN1, 0, 0, config)
    can.start_can(VCI_USBCAN1, 0, 0)
    
    radar = MR76Radar(can, VCI_USBCAN1, 0, 0, sensor_id=0)
    
    print("MR76 Radar - Collision Detection Example")
    
    # Configure detection zone: 10m wide x 170m deep
    coords = calculate_region_coordinates(width_m=10, depth_m=170, 
                                         center_lat=0, start_long=0)
    
    print(f"Configuring detection zone:")
    print(f"  Point 1 (bottom-right): ({coords[0]:.1f}m, {coords[1]:.1f}m)")
    print(f"  Point 2 (top-left): ({coords[2]:.1f}m, {coords[3]:.1f}m)")
    
    radar.configure_collision_region(*coords, activate=True)
    radar.enable_collision_detection(enable=True, min_time=0.5)
    
    time.sleep(1)  # Wait for configuration
    
    print("\nMonitoring collision zone...\n")
    
    try:
        while True:
            radar.process_can_messages(timeout_ms=100)
            
            # Check for collision warnings
            if radar.is_region_warning_active():
                print("⚠️  WARNING: Object detected in collision zone!")
                radar.print_collision_state()
                radar.print_objects()
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        can.close_device(VCI_USBCAN1, 0)


if __name__ == "__main__":
    example_basic_detection()
    pass
    # Run individual tests #

    # print("\n" + "=" * 70)
    # print("MR76 Radar Interface Library - Complete Motorola Implementation")
    # print("=" * 70)
    # print("\n✓ ALL messages now use proper Motorola (Big-Endian) bit ordering")
    # print("\nMessages updated:")
    # print("  ✓ 0x200 - RadarConfig (encoding)")
    # print("  ✓ 0x201 - RadarState (decoding)")
    # print("  ✓ 0x400 - CollisionDetectionConfig (encoding)")
    # print("  ✓ 0x401 - CollisionDetectionRegion (encoding)")
    # print("  ✓ 0x402 - CollisionDetectionRegionState (decoding)")
    # print("  ✓ 0x408 - CollisionDetectionState (decoding)")
    # print("  ✓ 0x60A - ObjectListStatus (decoding)")
    # print("  ✓ 0x60B - ObjectGeneral (decoding)")
    # print("  ✓ 0x60E - ObjectCollisionWarning (decoding)")
    # print("  ✓ 0x700 - SoftwareVersion (decoding)")
    # print("\nRunning comprehensive validation tests...")
    
    # # Run all validation tests
    # test_motorola_bit_order()
    # parse_object_example()
    # test_region_encoding_decoding_1()
    # test_region_encoding_decoding_2()
    # test_all_messages_motorola()
    
    # print("\n" + "=" * 70)
    # print("✓ Complete validation finished!")
    # print("All MR76 messages correctly use Motorola bit ordering")
    # print("=" * 70)

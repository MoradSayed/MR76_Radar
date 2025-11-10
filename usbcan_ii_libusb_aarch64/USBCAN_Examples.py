"""
Complete usage examples for ZLGCAN CAN interface library
This demonstrates how to use each method properly
"""

from USBCAN_Interface import *
import time

def example_1_basic_setup():
    """
    Example 1: Basic device setup and initialization
    """
    print("=" * 60)
    print("Example 1: Basic Device Setup")
    print("=" * 60)
    
    # Step 1: Create ControlCAN instance
    # Specify library path if not in default location
    # can = ControlCAN("ControlCAN.dll")  # Windows
    can = ControlCAN()  # Linux
    
    # Step 2: Open the device
    device_type = VCI_USBCAN2  # Using USBCAN2 device
    device_index = 0           # First device (0-based)
    reserved = 0               # Reserved parameter
    
    result = can.open_device(device_type, device_index, reserved)
    if result == STATUS_OK:
        print("✓ Device opened successfully")
    else:
        print("✗ Failed to open device")
        return
    
    # Step 3: Read board information
    result, board_info = can.read_board_info(device_type, device_index)
    if result == STATUS_OK:
        print(f"✓ Board Info:")
        print(f"  Hardware Version: {board_info.hw_Version}")
        print(f"  Firmware Version: {board_info.fw_Version}")
        print(f"  Driver Version: {board_info.dr_Version}")
        print(f"  CAN Channels: {board_info.can_Num}")
        print(f"  Serial Number: {board_info.str_Serial_Num.decode('utf-8')}")
        print(f"  Hardware Type: {board_info.str_hw_Type.decode('utf-8')}")
    
    # Step 4: Initialize CAN channel
    can_index = 0  # First CAN channel
    
    # Create initialization configuration
    init_config = VCI_INIT_CONFIG()
    init_config.AccCode = 0x00000000      # Acceptance code (accept all)
    init_config.AccMask = 0xFFFFFFFF      # Acceptance mask (accept all)
    init_config.Filter = 1                # Filter mode: 1=double filter
    init_config.Timing0 = 0x00            # Baud rate timing0
    init_config.Timing1 = 0x1C            # Baud rate timing1
    init_config.Mode = 0                  # Mode: 0=normal, 1=listen only
    # Common baud rates:
    # 1000Kbps: Timing0=0x00, Timing1=0x14
    # 500Kbps:  Timing0=0x00, Timing1=0x1C
    # 250Kbps:  Timing0=0x01, Timing1=0x1C
    # 125Kbps:  Timing0=0x03, Timing1=0x1C
    
    result = can.init_can(device_type, device_index, can_index, init_config)
    if result == STATUS_OK:
        print("✓ CAN initialized successfully (500Kbps)")
    else:
        print("✗ Failed to initialize CAN")
        can.close_device(device_type, device_index)
        return
    
    # Step 5: Start CAN communication
    result = can.start_can(device_type, device_index, can_index)
    if result == STATUS_OK:
        print("✓ CAN started successfully")
    else:
        print("✗ Failed to start CAN")
    
    # Remember to close device when done
    can.close_device(device_type, device_index)
    print("✓ Device closed\n")


def example_2_send_standard_frame():
    """
    Example 2: Sending a standard CAN frame
    """
    print("=" * 60)
    print("Example 2: Send Standard CAN Frame")
    print("=" * 60)
    
    can = ControlCAN()
    device_type = VCI_USBCAN2
    device_index = 0
    can_index = 0
    
    # Open and initialize (simplified)
    can.open_device(device_type, device_index, 0)
    
    init_config = VCI_INIT_CONFIG()
    init_config.AccCode = 0x00000000
    init_config.AccMask = 0xFFFFFFFF
    init_config.Filter = 1
    init_config.Timing0 = 0x00
    init_config.Timing1 = 0x1C
    init_config.Mode = 0
    
    can.init_can(device_type, device_index, can_index, init_config)
    can.start_can(device_type, device_index, can_index)
    
    # Create a standard CAN frame
    frame = VCI_CAN_OBJ()
    frame.ID = 0x123              # CAN ID (standard: 0x000-0x7FF)
    frame.SendType = 0            # 0=normal send, 1=single send, 2=self-test
    frame.RemoteFlag = 0          # 0=data frame, 1=remote frame
    frame.ExternFlag = 0          # 0=standard frame, 1=extended frame
    frame.DataLen = 8             # Data length (0-8 bytes)
    
    # Set data bytes
    frame.Data[0] = 0x01
    frame.Data[1] = 0x02
    frame.Data[2] = 0x03
    frame.Data[3] = 0x04
    frame.Data[4] = 0x05
    frame.Data[5] = 0x06
    frame.Data[6] = 0x07
    frame.Data[7] = 0x08
    
    # Send the frame
    result = can.transmit(device_type, device_index, can_index, frame)
    if result == 1:  # Returns number of frames sent
        print("✓ Sent 1 standard frame:")
        print(f"  ID: 0x{frame.ID:03X}")
        print(f"  Data: {' '.join([f'{frame.Data[i]:02X}' for i in range(frame.DataLen)])}")
    else:
        print("✗ Failed to send frame")
    
    can.close_device(device_type, device_index)
    print()


def example_3_send_extended_frame():
    """
    Example 3: Sending an extended CAN frame
    """
    print("=" * 60)
    print("Example 3: Send Extended CAN Frame")
    print("=" * 60)
    
    can = ControlCAN()
    device_type = VCI_USBCAN2
    device_index = 0
    can_index = 0
    
    # Setup (simplified)
    can.open_device(device_type, device_index, 0)
    init_config = VCI_INIT_CONFIG()
    init_config.AccCode = 0x00000000
    init_config.AccMask = 0xFFFFFFFF
    init_config.Filter = 1
    init_config.Timing0 = 0x00
    init_config.Timing1 = 0x1C
    init_config.Mode = 0
    can.init_can(device_type, device_index, can_index, init_config)
    can.start_can(device_type, device_index, can_index)
    
    # Create an extended CAN frame
    frame = VCI_CAN_OBJ()
    frame.ID = 0x12345678         # Extended CAN ID (0x00000000-0x1FFFFFFF)
    frame.SendType = 0            # Normal send
    frame.RemoteFlag = 0          # Data frame
    frame.ExternFlag = 1          # Extended frame (29-bit ID)
    frame.DataLen = 4             # 4 bytes of data
    
    frame.Data[0] = 0xAA
    frame.Data[1] = 0xBB
    frame.Data[2] = 0xCC
    frame.Data[3] = 0xDD
    
    result = can.transmit(device_type, device_index, can_index, frame)
    if result == 1:
        print("✓ Sent 1 extended frame:")
        print(f"  ID: 0x{frame.ID:08X} (29-bit)")
        print(f"  Data: {' '.join([f'{frame.Data[i]:02X}' for i in range(frame.DataLen)])}")
    else:
        print("✗ Failed to send frame")
    
    can.close_device(device_type, device_index)
    print()


def example_4_send_multiple_frames():
    """
    Example 4: Sending multiple CAN frames at once
    """
    print("=" * 60)
    print("Example 4: Send Multiple CAN Frames")
    print("=" * 60)
    
    can = ControlCAN()
    device_type = VCI_USBCAN2
    device_index = 0
    can_index = 0
    
    # Setup
    can.open_device(device_type, device_index, 0)
    init_config = VCI_INIT_CONFIG()
    init_config.AccCode = 0x00000000
    init_config.AccMask = 0xFFFFFFFF
    init_config.Filter = 1
    init_config.Timing0 = 0x00
    init_config.Timing1 = 0x1C
    init_config.Mode = 0
    can.init_can(device_type, device_index, can_index, init_config)
    can.start_can(device_type, device_index, can_index)
    
    # Create multiple frames
    frames = []
    
    # Frame 1
    frame1 = VCI_CAN_OBJ()
    frame1.ID = 0x100
    frame1.ExternFlag = 0
    frame1.RemoteFlag = 0
    frame1.DataLen = 2
    frame1.Data[0] = 0x11
    frame1.Data[1] = 0x22
    frames.append(frame1)
    
    # Frame 2
    frame2 = VCI_CAN_OBJ()
    frame2.ID = 0x200
    frame2.ExternFlag = 0
    frame2.RemoteFlag = 0
    frame2.DataLen = 4
    frame2.Data[0] = 0x33
    frame2.Data[1] = 0x44
    frame2.Data[2] = 0x55
    frame2.Data[3] = 0x66
    frames.append(frame2)
    
    # Frame 3
    frame3 = VCI_CAN_OBJ()
    frame3.ID = 0x300
    frame3.ExternFlag = 0
    frame3.RemoteFlag = 0
    frame3.DataLen = 8
    for i in range(8):
        frame3.Data[i] = 0x10 + i
    frames.append(frame3)
    
    # Send all frames
    result = can.transmit(device_type, device_index, can_index, frames)
    print(f"✓ Sent {result} frames out of {len(frames)} attempted")
    
    for i, frame in enumerate(frames[:result]):
        print(f"  Frame {i+1}: ID=0x{frame.ID:03X}, "
              f"Data={' '.join([f'{frame.Data[j]:02X}' for j in range(frame.DataLen)])}")
    
    can.close_device(device_type, device_index)
    print()


def example_5_receive_frames():
    """
    Example 5: Receiving CAN frames
    """
    print("=" * 60)
    print("Example 5: Receive CAN Frames")
    print("=" * 60)
    
    can = ControlCAN()
    device_type = VCI_USBCAN2
    device_index = 0
    can_index = 0
    
    # Setup
    can.open_device(device_type, device_index, 0)
    init_config = VCI_INIT_CONFIG()
    init_config.AccCode = 0x00000000
    init_config.AccMask = 0xFFFFFFFF
    init_config.Filter = 1
    init_config.Timing0 = 0x00
    init_config.Timing1 = 0x1C
    init_config.Mode = 0
    can.init_can(device_type, device_index, can_index, init_config)
    can.start_can(device_type, device_index, can_index)
    
    print("Waiting for CAN frames (10 seconds)...")
    
    start_time = time.time()
    total_received = 0
    
    while time.time() - start_time < 10:
        # Check how many frames are in the buffer
        num_in_buffer = can.get_receive_num(device_type, device_index, can_index)
        
        if num_in_buffer > 0:
            # Receive up to 10 frames at a time
            # wait_time: -1=wait forever, 0=no wait, >0=wait milliseconds
            count, frames = can.receive(device_type, device_index, can_index, 
                                       min(10, num_in_buffer), wait_time=0)
            
            for frame in frames:
                total_received += 1
                frame_type = "EXT" if frame.ExternFlag else "STD"
                remote = "RTR" if frame.RemoteFlag else "DAT"
                
                print(f"  Frame {total_received}:")
                print(f"    ID: 0x{frame.ID:08X if frame.ExternFlag else frame.ID:03X} [{frame_type}] [{remote}]")
                print(f"    DLC: {frame.DataLen}")
                
                if frame.RemoteFlag == 0:  # Data frame
                    data_str = ' '.join([f'{frame.Data[i]:02X}' for i in range(frame.DataLen)])
                    print(f"    Data: {data_str}")
                
                if frame.TimeFlag == 1:  # Hardware timestamp available
                    print(f"    Timestamp: {frame.TimeStamp} ms")
        
        time.sleep(0.1)  # Small delay to prevent CPU spinning
    
    print(f"\n✓ Total frames received: {total_received}")
    can.close_device(device_type, device_index)
    print()


def example_6_remote_frame():
    """
    Example 6: Sending a remote frame (RTR)
    """
    print("=" * 60)
    print("Example 6: Send Remote Frame (RTR)")
    print("=" * 60)
    
    can = ControlCAN()
    device_type = VCI_USBCAN2
    device_index = 0
    can_index = 0
    
    # Setup
    can.open_device(device_type, device_index, 0)
    init_config = VCI_INIT_CONFIG()
    init_config.AccCode = 0x00000000
    init_config.AccMask = 0xFFFFFFFF
    init_config.Filter = 1
    init_config.Timing0 = 0x00
    init_config.Timing1 = 0x1C
    init_config.Mode = 0
    can.init_can(device_type, device_index, can_index, init_config)
    can.start_can(device_type, device_index, can_index)
    
    # Create a remote frame
    frame = VCI_CAN_OBJ()
    frame.ID = 0x456
    frame.SendType = 0
    frame.RemoteFlag = 1          # Remote frame (request data)
    frame.ExternFlag = 0          # Standard frame
    frame.DataLen = 8             # Expected data length
    # Note: Data array is not used for remote frames
    
    result = can.transmit(device_type, device_index, can_index, frame)
    if result == 1:
        print("✓ Sent remote frame:")
        print(f"  ID: 0x{frame.ID:03X}")
        print(f"  Requested DLC: {frame.DataLen}")
    else:
        print("✗ Failed to send remote frame")
    
    can.close_device(device_type, device_index)
    print()


def example_7_error_handling():
    """
    Example 7: Error handling and status checking
    """
    print("=" * 60)
    print("Example 7: Error Handling and Status Checking")
    print("=" * 60)
    
    can = ControlCAN()
    device_type = VCI_USBCAN2
    device_index = 0
    can_index = 0
    
    # Open device
    result = can.open_device(device_type, device_index, 0)
    if result != STATUS_OK:
        print("✗ Failed to open device - check connection!")
        return
    
    # Initialize and start
    init_config = VCI_INIT_CONFIG()
    init_config.AccCode = 0x00000000
    init_config.AccMask = 0xFFFFFFFF
    init_config.Filter = 1
    init_config.Timing0 = 0x00
    init_config.Timing1 = 0x1C
    init_config.Mode = 0
    can.init_can(device_type, device_index, can_index, init_config)
    can.start_can(device_type, device_index, can_index)
    
    # Read CAN status
    result, status = can.read_can_status(device_type, device_index, can_index)
    if result == STATUS_OK:
        print("✓ CAN Status:")
        print(f"  Error Interrupt: {status.ErrInterrupt}")
        print(f"  Mode Register: 0x{status.regMode:02X}")
        print(f"  Status Register: 0x{status.regStatus:02X}")
        print(f"  RX Error Counter: {status.regRECounter}")
        print(f"  TX Error Counter: {status.regTECounter}")
    
    # Read error information
    result, err_info = can.read_err_info(device_type, device_index, can_index)
    if result == STATUS_OK:
        print("\n✓ Error Info:")
        
        if err_info.ErrCode == 0:
            print("  No errors detected")
        else:
            print(f"  Error Code: 0x{err_info.ErrCode:04X}")
            
            if err_info.ErrCode & ERR_CAN_OVERFLOW:
                print("  - CAN controller FIFO overflow")
            if err_info.ErrCode & ERR_CAN_ERRALARM:
                print("  - CAN controller error alarm")
            if err_info.ErrCode & ERR_CAN_PASSIVE:
                print("  - CAN controller passive error")
            if err_info.ErrCode & ERR_CAN_LOSE:
                print("  - CAN controller arbitration lost")
            if err_info.ErrCode & ERR_CAN_BUSERR:
                print("  - CAN controller bus error")
            
            if err_info.ErrCode & ERR_DEVICEOPEN:
                print("  - Device open error")
            if err_info.ErrCode & ERR_DEVICENOTOPEN:
                print("  - Device not opened")
            if err_info.ErrCode & ERR_BUFFEROVERFLOW:
                print("  - Buffer overflow")
    
    can.close_device(device_type, device_index)
    print()


def example_8_buffer_management():
    """
    Example 8: Buffer management
    """
    print("=" * 60)
    print("Example 8: Buffer Management")
    print("=" * 60)
    
    can = ControlCAN()
    device_type = VCI_USBCAN2
    device_index = 0
    can_index = 0
    
    # Setup
    can.open_device(device_type, device_index, 0)
    init_config = VCI_INIT_CONFIG()
    init_config.AccCode = 0x00000000
    init_config.AccMask = 0xFFFFFFFF
    init_config.Filter = 1
    init_config.Timing0 = 0x00
    init_config.Timing1 = 0x1C
    init_config.Mode = 0
    can.init_can(device_type, device_index, can_index, init_config)
    can.start_can(device_type, device_index, can_index)
    
    # Check receive buffer
    num_frames = can.get_receive_num(device_type, device_index, can_index)
    print(f"Frames in receive buffer: {num_frames}")
    
    # Clear the receive buffer
    result = can.clear_buffer(device_type, device_index, can_index)
    if result == STATUS_OK:
        print("✓ Receive buffer cleared")
        
        num_frames = can.get_receive_num(device_type, device_index, can_index)
        print(f"Frames in buffer after clear: {num_frames}")
    
    can.close_device(device_type, device_index)
    print()


def example_9_reset_can():
    """
    Example 9: Resetting CAN controller
    """
    print("=" * 60)
    print("Example 9: Reset CAN Controller")
    print("=" * 60)
    
    can = ControlCAN()
    device_type = VCI_USBCAN2
    device_index = 0
    can_index = 0
    
    # Setup
    can.open_device(device_type, device_index, 0)
    init_config = VCI_INIT_CONFIG()
    init_config.AccCode = 0x00000000
    init_config.AccMask = 0xFFFFFFFF
    init_config.Filter = 1
    init_config.Timing0 = 0x00
    init_config.Timing1 = 0x1C
    init_config.Mode = 0
    can.init_can(device_type, device_index, can_index, init_config)
    can.start_can(device_type, device_index, can_index)
    
    print("CAN controller running...")
    
    # Reset CAN controller (stops communication)
    result = can.reset_can(device_type, device_index, can_index)
    if result == STATUS_OK:
        print("✓ CAN controller reset")
        print("  Communication stopped, need to call start_can() to resume")
    
    # Restart CAN
    result = can.start_can(device_type, device_index, can_index)
    if result == STATUS_OK:
        print("✓ CAN controller restarted")
    
    can.close_device(device_type, device_index)
    print()


def example_10_complete_workflow():
    """
    Example 10: Complete workflow - Open, Send, Receive, Close
    """
    print("=" * 60)
    print("Example 10: Complete Workflow")
    print("=" * 60)
    
    can = ControlCAN()
    device_type = VCI_USBCAN2
    device_index = 0
    can_index = 0
    
    try:
        # 1. Open device
        print("Step 1: Opening device...")
        if can.open_device(device_type, device_index, 0) != STATUS_OK:
            raise Exception("Failed to open device")
        print("✓ Device opened")
        
        # 2. Read board info
        print("\nStep 2: Reading board info...")
        result, board_info = can.read_board_info(device_type, device_index)
        if result == STATUS_OK:
            print(f"✓ Board: {board_info.str_hw_Type.decode('utf-8')}")
            print(f"  Serial: {board_info.str_Serial_Num.decode('utf-8')}")
        
        # 3. Initialize CAN
        print("\nStep 3: Initializing CAN (500Kbps)...")
        init_config = VCI_INIT_CONFIG()
        init_config.AccCode = 0x00000000
        init_config.AccMask = 0xFFFFFFFF
        init_config.Filter = 1
        init_config.Timing0 = 0x00
        init_config.Timing1 = 0x1C
        init_config.Mode = 0
        
        if can.init_can(device_type, device_index, can_index, init_config) != STATUS_OK:
            raise Exception("Failed to initialize CAN")
        print("✓ CAN initialized")
        
        # 4. Start CAN
        print("\nStep 4: Starting CAN...")
        if can.start_can(device_type, device_index, can_index) != STATUS_OK:
            raise Exception("Failed to start CAN")
        print("✓ CAN started")
        
        # 5. Clear buffer
        print("\nStep 5: Clearing receive buffer...")
        can.clear_buffer(device_type, device_index, can_index)
        print("✓ Buffer cleared")
        
        # 6. Send frame
        print("\nStep 6: Sending test frame...")
        frame = VCI_CAN_OBJ()
        frame.ID = 0x123
        frame.ExternFlag = 0
        frame.RemoteFlag = 0
        frame.DataLen = 8
        for i in range(8):
            frame.Data[i] = i + 1
        
        if can.transmit(device_type, device_index, can_index, frame) == 1:
            print(f"✓ Sent frame: ID=0x{frame.ID:03X}")
        
        # 7. Wait and receive
        print("\nStep 7: Waiting for frames (5 seconds)...")
        time.sleep(5)
        
        num_in_buffer = can.get_receive_num(device_type, device_index, can_index)
        print(f"  Frames in buffer: {num_in_buffer}")
        
        if num_in_buffer > 0:
            count, frames = can.receive(device_type, device_index, can_index, 
                                       num_in_buffer, wait_time=100)
            print(f"✓ Received {count} frame(s)")
            for i, rx_frame in enumerate(frames):
                print(f"  Frame {i+1}: ID=0x{rx_frame.ID:03X}, "
                      f"DLC={rx_frame.DataLen}")
        
        # 8. Check status
        print("\nStep 8: Checking CAN status...")
        result, status = can.read_can_status(device_type, device_index, can_index)
        if result == STATUS_OK:
            print(f"✓ RX Errors: {status.regRECounter}, TX Errors: {status.regTECounter}")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
    
    finally:
        # 9. Always close device
        print("\nStep 9: Closing device...")
        can.close_device(device_type, device_index)
        print("✓ Device closed")
    
    print("\n" + "=" * 60)


# Run all examples
if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("ZLGCAN CAN Interface - Complete Usage Examples")
    print("=" * 60 + "\n")
    
    # Note: These examples assume you have a ZLGCAN device connected
    # Comment out examples as needed
    
    try:
        example_1_basic_setup()
        example_2_send_standard_frame()
        example_3_send_extended_frame()
        example_4_send_multiple_frames()
        example_5_receive_frames()
        example_6_remote_frame()
        example_7_error_handling()
        example_8_buffer_management()
        example_9_reset_can()
        example_10_complete_workflow()
        
    except Exception as e:
        print(f"\n✗ Error running examples: {e}")
        print("Make sure:")
        print("  1. ZLGCAN device is connected")
        print("  2. ControlCAN library (DLL/SO) is in the correct path")
        print("  3. Device drivers are installed")
    
    print("\n" + "=" * 60)
    print("Examples complete!")
    print("=" * 60)

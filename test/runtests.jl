import MyCobot
import MyCobot.ProtocolCode as ProtocolCode

using Test

@testset "Frame Handling" begin
    # Test prepare_frame
    frame = MyCobot.prepare_frame(ProtocolCode.GET_ANGLES)
    @test frame == [0xFE, 0xFE, 0x02, 0x20, 0xFA]

    # Test extract_all_frames
    response = [0xFE, 0xFE, 0x02, 0x20, 0xFA, 0xFE, 0xFE, 0x03, 0x20, 0x01, 0xFA]
    frames = MyCobot.extract_all_frames(response)
    @test frames == [[0xFE, 0xFE, 0x02, 0x20, 0xFA], [0xFE, 0xFE, 0x03, 0x20, 0x01, 0xFA]]
end

@testset "Motion Mode" begin
    # Test set_fresh_mode
    frame = MyCobot.prepare_frame(ProtocolCode.SET_FRESH_MODE, [0x01])
    @test frame == [0xFE, 0xFE, 0x03, 0x16, 0x01, 0xFA]

    frame = MyCobot.prepare_frame(ProtocolCode.SET_FRESH_MODE, [0x00])
    @test frame == [0xFE, 0xFE, 0x03, 0x16, 0x00, 0xFA]
end

@testset "Set Color" begin
    # Test set_color frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.SET_COLOR, [0x00, 0x00, 0xFF])  # Blue
    @test frame == [0xFE, 0xFE, 0x05, 0x6A, 0x00, 0x00, 0xFF, 0xFA]

    frame = MyCobot.prepare_frame(ProtocolCode.SET_COLOR, [0xFF, 0x00, 0x00])  # Red
    @test frame == [0xFE, 0xFE, 0x05, 0x6A, 0xFF, 0x00, 0x00, 0xFA]

    frame = MyCobot.prepare_frame(ProtocolCode.SET_COLOR, [0x00, 0xFF, 0x00])  # Green
    @test frame == [0xFE, 0xFE, 0x05, 0x6A, 0x00, 0xFF, 0x00, 0xFA]
end

@testset "Gripper Control" begin
    # Test set_gripper_state frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.SET_GRIPPER_STATE, [0x00, 0x32])  # Open at speed 50
    @test frame == [0xFE, 0xFE, 0x04, 0x66, 0x00, 0x32, 0xFA]

    # Test set_gripper_value frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.SET_GRIPPER_VALUE, [0x32, 0x14])  # 50% open at speed 20
    @test frame == [0xFE, 0xFE, 0x04, 0x67, 0x32, 0x14, 0xFA]

    # Test set_gripper_calibration frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.SET_GRIPPER_CALIBRATION)
    @test frame == [0xFE, 0xFE, 0x02, 0x68, 0xFA]
end

@testset "Servo Control" begin
    # Test is_servo_powered frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.IS_SERVO_POWERED, [0x01])  # Check servo 1
    @test frame == [0xFE, 0xFE, 0x03, 0x50, 0x01, 0xFA]

    # Test are_all_servos_powered frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.ARE_ALL_SERVOS_POWERED)
    @test frame == [0xFE, 0xFE, 0x02, 0x51, 0xFA]

    # Test set_servo_parameter frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.SET_SERVO_PARAMETER, [0x01, 0x15, 0x0F])  # Set position P for servo 1 to 15
    @test frame == [0xFE, 0xFE, 0x05, 0x52, 0x01, 0x15, 0x0F, 0xFA]

    # Test read_servo_parameter frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.READ_SERVO_PARAMETER, [0x01, 0x15])  # Read position P for servo 1
    @test frame == [0xFE, 0xFE, 0x04, 0x53, 0x01, 0x15, 0xFA]

    # Test set_servo_zero frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.SET_SERVO_ZERO, [0x01])  # Set zero for servo 1
    @test frame == [0xFE, 0xFE, 0x03, 0x54, 0x01, 0xFA]

    # Test brake_servo frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.BRAKE_SERVO, [0x01])  # Brake servo 1
    @test frame == [0xFE, 0xFE, 0x03, 0x55, 0x01, 0xFA]

    # Test power_off_servo frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.POWER_OFF_SERVO, [0x01])  # Power off servo 1
    @test frame == [0xFE, 0xFE, 0x03, 0x56, 0x01, 0xFA]

    # Test power_on_servo frame construction
    frame = MyCobot.prepare_frame(ProtocolCode.POWER_ON_SERVO, [0x01])  # Power on servo 1
    @test frame == [0xFE, 0xFE, 0x03, 0x57, 0x01, 0xFA]
end

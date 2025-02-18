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

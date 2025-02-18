"""
    print_servo_data(sp::LibSerialPort.SerialPort)

Print the servo data for all servos.
"""
function print_servo_data(sp::LibSerialPort.SerialPort)
    print("                Servo number")
    for servo_i in 1:6
        print("    J", servo_i)
    end
    println()
    println()

    print("              LED alarm (20)")
    for servo_i in 1:6
        value = MyCobot.read_servo_parameter(sp, servo_i, 20)
        print("    ", value)
    end
    println()

    print("  Proportional gain - P (21)")
    for servo_i in 1:6
        value = MyCobot.read_servo_parameter(sp, servo_i, 21)
        print("    ", value)
    end
    println()

    print("    Derivative gain - D (22)")
    for servo_i in 1:6
        value = MyCobot.read_servo_parameter(sp, servo_i, 22)
        print("     ", value)
    end
    println()

    print("      Integral gain - I (23)")
    for servo_i in 1:6
        value = MyCobot.read_servo_parameter(sp, servo_i, 23)
        print("     ", value)
    end
    println()

    print("Minimum starting torque (24)")
    for servo_i in 1:6
        value = MyCobot.read_servo_parameter(sp, servo_i, 24)
        print("     ", value)
    end
    println()

    print("                      ? (25)")
    for servo_i in 1:6
        value = MyCobot.read_servo_parameter(sp, servo_i, 25)
        # print("     ", value)
    end
    println()

    print("    CW insensitive zone (26)")
    for servo_i in 1:6
        value = MyCobot.read_servo_parameter(sp, servo_i, 26)
        print("     ", value)
    end
    println()

    print("   CCW insensitive zone (27)")
    for servo_i in 1:6
        value = MyCobot.read_servo_parameter(sp, servo_i, 27)
        print("     ", value)
    end
    println()
end

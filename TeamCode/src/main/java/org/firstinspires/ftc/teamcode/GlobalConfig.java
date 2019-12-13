package org.firstinspires.ftc.teamcode;

public final class GlobalConfig {
    // Unit conversions
    public final static double MM_PER_INCH = 25.4, CM_PER_INCH = 2.54;

    // Bot and Hardware Measurements
    public final static double TICKS_PER_MOTOR_REVOLUTION = 537.6, MECANUM_CIRCUMFERENCE_MM = 100, MECANUM_CIRCUMFERENCE_IN = MECANUM_CIRCUMFERENCE_MM / MM_PER_INCH;

    // Ticks
    public final static int TICKS_PER_INCH = (int) Math.round(MECANUM_CIRCUMFERENCE_IN / TICKS_PER_MOTOR_REVOLUTION);
    public final static int TICKS_PER_360 = 5200;
}

package org.firstinspires.ftc.teamcode;

public final class GlobalConfig {
    // Unit conversions
    public final static double MM_PER_INCH = 25.4, CM_PER_INCH = 2.54;

    // Bot and Hardware Measurements
    public final static double TICKS_PER_MOTOR_REVOLUTION = 537.6, MECANUM_CIRCUMFERENCE_MM = 100 * Math.PI;
    public final static double MECANUM_CIRCUMFERENCE_IN = MECANUM_CIRCUMFERENCE_MM / MM_PER_INCH;
    public final static double LIFT_PULLEY_DIAMETER_MM = 38, LIFT_PULLEY_RADIUS_MM = LIFT_PULLEY_DIAMETER_MM / 2;
    public final static double LIFE_STAGE_HEIGHT_IN = 5.5, INTAKE_STARTING_FLOOR_DISTANCE_MM = 5;

    // Stone measurements
    public final static double STONE_LENGTH_IN = 8, STONE_WIDTH_IN = 4, STONE_HEIGHT_IN = 5;

    // Ticks
    public final static int TICKS_PER_INCH = (int) Math.round((TICKS_PER_MOTOR_REVOLUTION / MECANUM_CIRCUMFERENCE_IN) * 1.5);
    public final static int TICKS_PER_360 = 2850;

    // Left side claw positions
    public final static double LEFT_SIDE_CLAW_ARM_UP = 0.26, LEFT_SIDE_CLAW_ARM_DOWN = 0;
    public final static double LEFT_SIDE_CLAW_FINGER_OPEN = 0.7, LEFT_SIDE_CLAW_FINGER_CLOSE = 0;
}

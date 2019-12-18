package org.firstinspires.ftc.teamcode.teleop;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;

@TeleOp(name = "Main TeleOp", group = "A") // Group is A to ensure this is at the top of the list
public class Main extends OpMode {

    private Servo foundationServo;
    private MecanumDrive mecanumDrive;
    private CRServo intakeServoLeft, intakeServoRight;
    private DcMotor motorFL, motorFR, motorBL, motorBR, motorSlideLeft, motorSlideRight;
    private boolean isDriveSlowMode = false;

    @Override
    public void init() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSlideLeft = hardwareMap.dcMotor.get("liftLeft");
        motorSlideRight = hardwareMap.dcMotor.get("liftRight");

        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeServoLeft = hardwareMap.crservo.get("intakeLeft");
        intakeServoRight = hardwareMap.crservo.get("intakeRight");

        intakeServoRight.setDirection(DcMotorSimple.Direction.REVERSE);

        foundationServo = hardwareMap.servo.get("foundationMover");
        foundationServo.setPosition(0.15); // Reset position

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, GlobalConfig.TICKS_PER_INCH, GlobalConfig.TICKS_PER_360);

        motorSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // MOVEMENT (DRIVING/STRAFING AND ROTATION
        double strafe_x = -gamepad1.left_stick_x, strafe_y = gamepad1.left_stick_y, rotate_power = -gamepad1.right_stick_x;

        if (isDriveSlowMode) {
            strafe_x /= 3;
            strafe_y /= 3;
            rotate_power /= 3;
        }

        Coordinate strafe = Coordinate.fromXY(strafe_x, strafe_y);
        mecanumDrive.setStrafe(strafe);
        mecanumDrive.setRotationPower(rotate_power);

        // FOUNDATION MOVER
        if (gamepad1.left_bumper)
            foundationServo.setPosition(0.15);
        else if (gamepad1.right_bumper)
            foundationServo.setPosition(0.69);

        // INTAKE AND OUTPUT
        double intakeServoPower = 0;

        if (gamepad2.left_trigger > 0)
            intakeServoPower = -gamepad2.left_trigger;
        if (gamepad2.right_trigger > 0)
            intakeServoPower = gamepad2.right_trigger;

        intakeServoLeft.setPower(intakeServoPower);
        intakeServoRight.setPower(intakeServoPower);

        // ROTATE STONES
        // Left and right d-pad spins servos on intake both the same way to rotate stones
        if(gamepad2.dpad_left)
            intakeServoPower = 0.5;
        if(gamepad2.dpad_right)
            intakeServoPower = -0.5;

        intakeServoLeft.setPower(intakeServoPower);
        intakeServoRight.setPower(intakeServoPower);

        // SLIDES
        double liftPower = 0;

        if (gamepad2.right_stick_y != 0) {
            // Slower
            if (gamepad2.right_stick_y > 0)
                liftPower = -0.6;
            else
                liftPower = 0.6;
        } else if (gamepad2.left_stick_y != 0) {
            liftPower = - 0.1 * gamepad2.left_stick_y;
        }

        motorSlideLeft.setPower(liftPower);
        motorSlideRight.setPower(liftPower);

        if (gamepad1.a)
            isDriveSlowMode = !isDriveSlowMode;

        telemetry.addData("Slow Mode", isDriveSlowMode);
        telemetry.addData("Move Coordinate X", strafe_x);
        telemetry.addData("Move Coordinate Y", strafe_y);
        telemetry.addData("Rotation Power", rotate_power);
        telemetry.update();
    }
}

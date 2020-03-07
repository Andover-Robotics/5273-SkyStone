package org.firstinspires.ftc.teamcode.autonomous;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.vision.SkystoneDetector;

public class AutonomousMaster extends LinearOpMode {

    protected MecanumDrive mecanumDrive;
    protected SkystoneDetector skystoneDetector;
    private CRServo intakeServoLeft, intakeServoRight;
    protected Servo foundationServoLeft, foundationServoRight;
    protected Servo sideClawArmLeft, sideClawFingerLeft, sideClawArmRight, sideClawFingerRight;
    private DcMotor motorFL, motorFR, motorBL, motorBR, motorSlideLeft, motorSlideRight;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStartWithPings();

        telemetry.addData("Status", "STARTING");
        telemetry.update();
    }

    protected void checkForStop() throws InterruptedException {
        if (isStopRequested())
            throw new InterruptedException();
    }

    protected void initialize() {
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorSlideLeft = hardwareMap.dcMotor.get("liftLeft");
        motorSlideRight = hardwareMap.dcMotor.get("liftRight");
        motorSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] motors = {motorFL, motorFR, motorBL, motorBR}, slideMotors = {motorSlideLeft, motorSlideRight};

        // Adjust the tolerances and PID coefficients of motors to prevent micro-adjustments after movement
        for (DcMotor motor : motors) {
            DcMotorEx motorEX = (DcMotorEx) motor;
            motorEX.setTargetPositionTolerance((int) (0.8 * GlobalConfig.TICKS_PER_INCH + 0.5));
            PIDFCoefficients coefficients = motorEX.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            motorEX.setVelocityPIDFCoefficients(coefficients.p + 0.1, coefficients.i + 0.5, coefficients.d, coefficients.f + 0.2);
        }

        // Adjust the slide motors separately because they require more precision
        for (DcMotor motor : slideMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, GlobalConfig.TICKS_PER_INCH, GlobalConfig.TICKS_PER_360);
        mecanumDrive.setDefaultDrivePower(0.2);

        foundationServoLeft = hardwareMap.servo.get("foundationMoverLeft");
        foundationServoRight = hardwareMap.servo.get("foundationMoverRight");

        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_UP);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_UP);

        sideClawArmLeft = hardwareMap.servo.get("sideClawArmLeft");
        sideClawFingerLeft = hardwareMap.servo.get("sideClawFingerLeft");

        sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
        sideClawFingerLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_FINGER_OPEN);

        sideClawArmRight = hardwareMap.servo.get("sideClawArmRight");
        sideClawFingerRight = hardwareMap.servo.get("sideClawFingerRight");

        sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        sideClawFingerRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_FINGER_OPEN);

        skystoneDetector = new SkystoneDetector(hardwareMap, 300, 230, 110);
        skystoneDetector.start();

        intakeServoLeft = hardwareMap.crservo.get("intakeLeft");
        intakeServoRight = hardwareMap.crservo.get("intakeRight");
        intakeServoRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Waits for the OpMode to be run while sending messages between the phones
    // This avoids timeouts and resolves a common bug that often results in loss of connection
    private void waitForStartWithPings() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "WAITING");
            telemetry.addData("Time", System.currentTimeMillis());
            telemetry.update();
        }

        telemetry.update();
    }

    protected void driveForwards(double distanceInInches) {
        driveForwards(distanceInInches, mecanumDrive.getDefaultDrivePower());
    }

    protected void driveForwards(double distanceInInches, double power) {
        if (distanceInInches < 0)
            mecanumDrive.driveBackwards(-distanceInInches, power);
        else
            mecanumDrive.driveForwards(distanceInInches, power);
    }

    protected void driveBackwards(double distanceInInches) {
        driveBackwards(distanceInInches, mecanumDrive.getDefaultDrivePower());
    }

    protected void driveBackwards(double distanceInInches, double power) {
        driveForwards(-distanceInInches, power);
    }

    protected void strafeRight(double distanceInInches) {
        strafeRight(distanceInInches, mecanumDrive.getDefaultDrivePower());
    }

    protected void strafeRight(double distanceInInches, double power) {
        if (distanceInInches < 0)
            mecanumDrive.strafeLeft(-distanceInInches, power);
        else
            mecanumDrive.strafeRight(distanceInInches, power);
    }


    protected void strafeLeft(double distanceInInches) {
        strafeLeft(distanceInInches, mecanumDrive.getDefaultDrivePower());
    }

    protected void strafeLeft(double distanceInInches, double power) {
        strafeRight(-distanceInInches, power);
    }

    protected void rotateCW(int degrees) {
        rotateCW(degrees, mecanumDrive.getDefaultDrivePower());
    }

    protected void rotateCW(int degrees, double power) {
        if (degrees < 0)
            mecanumDrive.rotateCounterClockwise(-degrees, power);
        else
            mecanumDrive.rotateClockwise(degrees, power);
    }

    protected void rotateCCW(int degrees) {
        rotateCCW(degrees, mecanumDrive.getDefaultDrivePower());
    }

    protected void rotateCCW(int degrees, double power) {
        rotateCW(-degrees, power);
    }

    protected void setLiftPower(double power) {
        motorSlideRight.setPower(power);
        motorSlideLeft.setPower(power);
    }

    protected void holdLiftLocation() {
        setLiftPower(0.1);
    }

    protected void setIntakePower(double power) {
        intakeServoRight.setPower(power);
        intakeServoLeft.setPower(power);
    }
}


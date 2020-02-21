package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.vision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.vision.SkystoneLocation;

import java.util.function.Function;

public class AutonomousMasterRoadrunner extends LinearOpMode {

    protected SampleMecanumDriveBase driveBase;
    protected SkystoneDetector skystoneDetector;
    private CRServo intakeServoLeft, intakeServoRight;
    protected Servo foundationServoLeft, foundationServoRight, sideClawArmLeft, sideClawFingerLeft, sideClawArmRight, sideClawFingerRight;
    protected SkystoneLocation lastStoneLocation;
    private DcMotor motorSlideLeft, motorSlideRight;


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

        driveBase = new SampleMecanumDriveREVOptimized(hardwareMap);

        motorSlideLeft = hardwareMap.dcMotor.get("liftLeft");
        motorSlideRight = hardwareMap.dcMotor.get("liftRight");
        motorSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] slideMotors = {motorSlideLeft, motorSlideRight};

        // Adjust the slide motors separately because they require more precision
        for (DcMotor motor : slideMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        foundationServoLeft = hardwareMap.servo.get("foundationMoverLeft");
        foundationServoRight = hardwareMap.servo.get("foundationMoverRight");

        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_UP);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_UP);

        sideClawArmLeft = hardwareMap.servo.get("sideClawArmLeft");
        sideClawFingerLeft = hardwareMap.servo.get("sideClawFingerLeft");

        sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
        sideClawFingerLeft.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);

        sideClawArmRight = hardwareMap.servo.get("sideClawArmRight");
        sideClawFingerRight = hardwareMap.servo.get("sideClawFingerRight");

        sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);

        skystoneDetector = new SkystoneDetector(hardwareMap, 330, 215, 90);
        skystoneDetector.start();

        intakeServoLeft = hardwareMap.crservo.get("intakeLeft");
        intakeServoRight = hardwareMap.crservo.get("intakeRight");
        intakeServoRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // Waits for the OpMode to be run while sending messages between the phones
    // This avoids timeouts and resolves a common bug that often results in loss of connection
    private void waitForStartWithPings() {
        while (!opModeIsActive() && !isStopRequested()) {
            lastStoneLocation = scanForStone();
        }

        telemetry.update();
    }

    protected void drive(Function<TrajectoryBuilder, BaseTrajectoryBuilder> trajectory) {
        driveBase.followTrajectorySync(trajectory.apply(driveBase.trajectoryBuilder()).build());
    }

    protected SkystoneLocation scanForStone() {
        skystoneDetector.setFlashLight(true);
        SkystoneLocation skystoneLocation = skystoneDetector.getSkystoneLocation();

        skystoneLocation = skystoneDetector.getSkystoneLocation();

        telemetry.addLine("Scanning...");
        telemetry.addData("Skystone Location", skystoneLocation);
        telemetry.update();

        return skystoneLocation;
    }

    protected SkystoneLocation swapSkystoneLocation(SkystoneLocation skystoneLocation) {
        switch (skystoneLocation) {
            case LEFT:
                return SkystoneLocation.RIGHT;
            case RIGHT:
                return SkystoneLocation.LEFT;
            default:
                return skystoneLocation;
        }
    }

    protected void setLiftPower(double power) {
        motorSlideRight.setPower(power);
        motorSlideLeft.setPower(power);
    }

    protected void holdLiftLocation() {
        setLiftPower(0.1);
    }

    protected void grabStone(RobotAlliance currentAlliance, int armLiftDelay) {
        if (currentAlliance == RobotAlliance.BLUE) {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        } else {
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerLeft.setPosition(GlobalConfig.SIDE_CLAW_FINGER_CLOSE);
            sleep(750);
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
        }

        sleep(armLiftDelay);
    }

    protected void placeStone(RobotAlliance currentAlliance, int armLiftDelay) {
        if (currentAlliance == RobotAlliance.BLUE) {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);
            sleep(500);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        } else {
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerLeft.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);
            sleep(500);
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
        }

        sleep(armLiftDelay);
    }
}


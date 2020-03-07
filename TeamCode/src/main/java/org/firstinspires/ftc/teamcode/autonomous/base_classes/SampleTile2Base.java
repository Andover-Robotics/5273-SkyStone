package org.firstinspires.ftc.teamcode.autonomous.base_classes;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;
import org.firstinspires.ftc.teamcode.vision.SkystoneLocation;

public abstract class SampleTile2Base extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        int allianceDistanceMultiplier = currentAlliance == RobotAlliance.RED ? 1 : -1;

        driveForwards(6, 0.5);
        rotateCW(90 * allianceDistanceMultiplier, 0.5);

        skystoneDetector.setFlashLight(true);

        mecanumDrive.setMovementPower(-0.75);

        sleep(500);

        mecanumDrive.setMovementPower(-0.4);

        sleep(500);

        mecanumDrive.stop();

        sleep(250); // Let the camera refocus

        SkystoneLocation skystoneLocation = skystoneDetector.getSkystoneLocation();

        while (skystoneLocation == null) {
            skystoneLocation = skystoneDetector.getSkystoneLocation();

            telemetry.addLine("Scanning...");
            telemetry.addData("Skystone Location", skystoneLocation);
            telemetry.update();

            idle();
        }

        if (currentAlliance == RobotAlliance.BLUE) {
            if (skystoneLocation == SkystoneLocation.LEFT)
                skystoneLocation = SkystoneLocation.RIGHT;
            else if (skystoneLocation == SkystoneLocation.RIGHT)
                skystoneLocation = SkystoneLocation.LEFT;
        }

        skystoneDetector.stop();

        driveForwards(3.5, 0.5);

        telemetry.addData("Skystone Location", skystoneLocation);
        telemetry.update();

        strafeLeft(24 * allianceDistanceMultiplier, 0.5);

        mecanumDrive.setMovementPower(-0.3);
        sleep(750);
        mecanumDrive.stop();

        double distanceToDriveForward = 4.5 * 24;


        switch (skystoneLocation) {
            case MIDDLE:
                driveForwards(8);
                distanceToDriveForward -= 8;
                break;
            case RIGHT:
                driveForwards(16);
                distanceToDriveForward -= 16;
                break;
            default:
                break;
        }

        if (currentAlliance == RobotAlliance.RED) {
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN);
            sleep(500);
            sideClawFingerLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
            sleep(500);
        } else {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(500);
            sideClawFingerRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
            sleep(500);
        }

        strafeRight(6 * allianceDistanceMultiplier, 0.5);

        driveForwards(distanceToDriveForward, 0.75);

        strafeLeft(12*allianceDistanceMultiplier,0.5);

        if (currentAlliance == RobotAlliance.RED) {
            sideClawArmLeft.setPosition((GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN + GlobalConfig.LEFT_SIDE_CLAW_ARM_UP) / 2);
            sleep(300);
            sideClawFingerLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_FINGER_OPEN);
        } else {
            sideClawArmRight.setPosition((GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN + GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP) / 2);
            sleep(300);
            sideClawFingerRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_FINGER_OPEN);
        }

        sleep(150);
        strafeLeft(2 * allianceDistanceMultiplier, 0.5);
        strafeRight(7 * allianceDistanceMultiplier, 0.5);

        if (currentAlliance == RobotAlliance.RED)
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
        else
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);

        driveBackwards(42,0.5);

        /*setLiftPower(0.6);
        sleep(250);
        holdLiftLocation();
        rotateCCW(90, 0.5);
        driveForwards(12, 0.5);
        // Latch on
        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_DOWN);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_DOWN);
        sleep(400);
        setLiftPower(0.0075);
        //mecanumDrive.setMovementPower(-0.65);
        driveBackwards(12,0.5);
        mecanumDrive.rotateClockwise(165 * allianceDistanceMultiplier, 0.5);
        sleep(350);
        setLiftPower(0);
        mecanumDrive.setMovementPower(0.65);
        sleep(350);
        setLiftPower(0.4);
        sleep(100);
        holdLiftLocation();
        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_UP);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_UP);
        mecanumDrive.driveBackwards(50,0.5);*/
        mecanumDrive.stop();
    }
}
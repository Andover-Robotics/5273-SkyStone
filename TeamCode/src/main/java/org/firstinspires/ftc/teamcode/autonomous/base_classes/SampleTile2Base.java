package org.firstinspires.ftc.teamcode.autonomous.base_classes;

import android.provider.Settings;

import com.andoverrobotics.core.utilities.Coordinate;

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

        driveBackwards(26, 0.4);

        sleep(250); // Let the camera refocus

        SkystoneLocation skystoneLocation = skystoneDetector.getSkystoneLocation();

        while (skystoneLocation == null) {
            skystoneLocation = skystoneDetector.getSkystoneLocation();

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

        double distanceToDriveForward = 4 * 24;


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

        if(currentAlliance == RobotAlliance.RED) {
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN);
            sleep(1000);
            sideClawFingerLeft.setPosition(GlobalConfig.SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
            sleep(1000);
        } else {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(1000);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
            sleep(1000);
        }

        strafeRight(12 * allianceDistanceMultiplier, 0.5);

        driveForwards(distanceToDriveForward, 0.75);
        strafeRight(30, 0.5);

        if(currentAlliance == RobotAlliance.RED) {
            sideClawArmLeft.setPosition((GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN+GlobalConfig.LEFT_SIDE_CLAW_ARM_UP)/2);
            sleep(1000);
            sideClawFingerLeft.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);
        } else {
            sideClawArmRight.setPosition((GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN+GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP)/2);
            sleep(1000);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);
        }
    }

}

package org.firstinspires.ftc.teamcode.autonomous.base_classes;

import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;
import org.firstinspires.ftc.teamcode.vision.SkystoneLocation;

public abstract class SampleTile2Base extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        mecanumDrive.driveForwards(16);

        sleep(200); // Let the camera refocus

        SkystoneLocation skystoneLocation = skystoneDetector.getSkystoneLocation();

        while (skystoneLocation == null) {
            skystoneLocation = skystoneDetector.getSkystoneLocation();

            telemetry.addData("Skystone Location", skystoneLocation);
            telemetry.update();

            idle();
        }

        telemetry.addData("Skystone Location", skystoneLocation);
        telemetry.update();

        int allianceDistanceMultiplier = currentAlliance == RobotAlliance.RED ? 1 : -1;

        switch (skystoneLocation) {
            case LEFT:
                mecanumDrive.driveBackwards(2 * allianceDistanceMultiplier);
                break;
            case MIDDLE:
                mecanumDrive.driveBackwards(10 * allianceDistanceMultiplier);
                break;
            case RIGHT:
                mecanumDrive.driveForwards(4 * allianceDistanceMultiplier);
                break;
        }

        mecanumDrive.rotateClockwise(90 * allianceDistanceMultiplier);
        mecanumDrive.driveForwards(24);
    }

}

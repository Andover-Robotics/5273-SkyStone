package org.firstinspires.ftc.teamcode.autonomous.base_classes;

import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;
import org.firstinspires.ftc.teamcode.vision.SkystoneLocation;

public abstract class SampleTile2Base extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        int allianceDistanceMultiplier = currentAlliance == RobotAlliance.RED ? 1 : -1;

        strafeRight(2 * allianceDistanceMultiplier);

        driveForwards(16);

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


        switch (skystoneLocation) {
            case LEFT:
                driveBackwards(2 * allianceDistanceMultiplier);
                break;
            case MIDDLE:
                driveBackwards(10 * allianceDistanceMultiplier);
                break;
            case RIGHT:
                driveForwards(4 * allianceDistanceMultiplier);
                break;
        }

        rotateCW(90 * allianceDistanceMultiplier);
        driveForwards(28);

        if (skystoneLocation != SkystoneLocation.RIGHT)
            rotateCCW(90 * allianceDistanceMultiplier);
        else
            rotateCW(90 * allianceDistanceMultiplier);
    }

}

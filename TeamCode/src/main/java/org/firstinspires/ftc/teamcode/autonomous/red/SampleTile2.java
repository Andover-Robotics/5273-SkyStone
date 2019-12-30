package org.firstinspires.ftc.teamcode.autonomous.red;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousMaster;
import org.firstinspires.ftc.teamcode.vision.SkystoneLocation;

public class SampleTile2 extends AutonomousMaster {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        mecanumDrive.driveForwards(12);

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
                mecanumDrive.driveForwards(3);
                break;
            case MIDDLE:
                mecanumDrive.driveBackwards(5);
                break;
            case RIGHT:
                mecanumDrive.driveForwards(0); // TODO: Measure this distance
                break;
        }


    }

}

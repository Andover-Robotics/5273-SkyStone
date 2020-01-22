package org.firstinspires.ftc.teamcode.autonomous.base_classes;

import com.andoverrobotics.core.utilities.Coordinate;

import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;
import org.firstinspires.ftc.teamcode.vision.SkystoneLocation;

public abstract class SampleTile2Base extends AutonomousBase {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        int allianceDistanceMultiplier = currentAlliance == RobotAlliance.RED ? 1 : -1;

        setLiftPower(0.35);
        sleep(250);
        holdLiftLocation();

        driveForwards(6, 0.5);
        rotateCCW(90, 0.5);

        skystoneDetector.setFlashLight(true);
        driveForwards(12 * allianceDistanceMultiplier, 0.35);

        if (currentAlliance == RobotAlliance.BLUE) {
            mecanumDrive.setMovementPower(-0.3);
            sleep(1000);
            mecanumDrive.stop();
        }

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

            driveForwards(1);
        }

        skystoneDetector.stop();

        if (currentAlliance == RobotAlliance.BLUE && skystoneLocation != SkystoneLocation.RIGHT)
            driveForwards(12, 0.5);


        telemetry.addData("Skystone Location", skystoneLocation);
        telemetry.update();


        switch (skystoneLocation) {
            case LEFT:
                setLiftPower(0.25);
                rotateCW(180, 0.5);
                holdLiftLocation();

                strafeLeft(20, 0.75);

                driveForwards(3.5);

                strafeLeft(14, 0.5);

                rotateCCW(30);
                setIntakePower(1);
                driveForwards(4);

                setLiftPower(-0.25);
                rotateCW(30);
                setLiftPower(0);
                sleep(1250);

                driveBackwards(4);

                setLiftPower(0.5);
                sleep(375);
                holdLiftLocation();

                setIntakePower(0);

                strafeRight(26, 0.75);
                rotateCCW(180, 0.5);

                setIntakePower(-0.25);
                sleep(500);
                setIntakePower(0);

                driveForwards(70, 0.8);

                break;
            case MIDDLE:
                driveBackwards(3 * allianceDistanceMultiplier, 0.5);

                rotateCW(90);

                driveForwards(20, 0.5);
                driveForwards(18);
                setLiftPower(0.5);
                driveBackwards(17.5, 0.4);
                holdLiftLocation();

                strafeLeft(11 * allianceDistanceMultiplier);
                driveForwards(11.5, 0.3);
                driveBackwards(2);

                setIntakePower(1);
                setLiftPower(-0.25);
                sleep(250);
                setLiftPower(0);
                strafeLeft(2, 0.5);
                sleep(1000);


                driveBackwards(5, 0.25);
                driveBackwards(25, 0.5);
                setLiftPower(0.5);
                sleep(250);
                holdLiftLocation();
                setIntakePower(0);

                rotateCW(80 * allianceDistanceMultiplier, 0.75);
                setIntakePower(-0.25);
                sleep(100);
                setIntakePower(0);
                driveForwards(76, 0.75);

                break;
            case RIGHT:
                if (currentAlliance == RobotAlliance.RED) {
                    driveForwards(4 * allianceDistanceMultiplier);

                    rotateCW(180);
                }

                strafeLeft(22 * allianceDistanceMultiplier, 0.5);

                mecanumDrive.setMovementPower(-0.2);
                sleep(currentAlliance == RobotAlliance.RED ? 1500 : 300);
                mecanumDrive.stop();

                setLiftPower(0.75);
                sleep(300);
                holdLiftLocation();

                strafeLeft(16.5 * allianceDistanceMultiplier, 0.5);

                if (currentAlliance == RobotAlliance.BLUE)
                    rotateCCW(15, 0.75);

                driveForwards(4);

                rotateCCW(35 * allianceDistanceMultiplier, 0.75);
                rotateCW(15 * allianceDistanceMultiplier, 0.5);

                driveForwards(6);
                driveBackwards(2.85);

                setLiftPower(-0.25);
                setIntakePower(1);
                sleep(500);
                setLiftPower(0);

                sleep(2000);

                driveBackwards(2);


                rotateCW(allianceDistanceMultiplier * 25);
                mecanumDrive.setMovementPower(-0.2);
                setLiftPower(0.5);
                sleep(750);
                holdLiftLocation();
                mecanumDrive.stop();
                setIntakePower(0);

                strafeRight(allianceDistanceMultiplier * 34, 0.5);

                mecanumDrive.setMovementPower(-0.2);
                setLiftPower(-0.25);
                setIntakePower(1);
                sleep(500);
                setLiftPower(0);
                sleep(500);
                mecanumDrive.stop();
                setIntakePower(0);

                setLiftPower(0.5);
                sleep(250);
                holdLiftLocation();

                driveForwards(1);

                driveForwards(80, 0.85);

                break;
        }


        mecanumDrive.setStrafe(Coordinate.fromXY(allianceDistanceMultiplier, 0), 0.25);
        sleep(500);
        mecanumDrive.stop();

        setLiftPower(0.45);
        setIntakePower(-1);
        sleep(500);
        holdLiftLocation();
        sleep(500);
        driveBackwards(8);
        setIntakePower(0);
        setLiftPower(-0.25);
        sleep(250);
        setLiftPower(0);

        driveBackwards(14);

        mecanumDrive.setStrafe(Coordinate.fromXY(allianceDistanceMultiplier, 0), 0.25);
        sleep(1000);
        mecanumDrive.stop();

    }

}

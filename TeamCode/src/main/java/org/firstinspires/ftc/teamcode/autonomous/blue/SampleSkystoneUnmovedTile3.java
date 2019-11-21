package org.firstinspires.ftc.teamcode.autonomous.blue;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousMaster;

public class SampleSkystoneUnmovedTile3 extends AutonomousMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        mecanumDrive.driveForwards(12);
        checkForStop();

        // TODO: Sample skystone

        mecanumDrive.rotateCounterClockwise(90);
        checkForStop();

        mecanumDrive.driveForwards(72);
        checkForStop();

        mecanumDrive.rotateClockwise(90);
        checkForStop();

        mecanumDrive.driveForwards(18);
        checkForStop();

        // TODO: Deposit stone into foundation

        mecanumDrive.driveBackwards(18);
        checkForStop();

        mecanumDrive.rotateClockwise(180);
        checkForStop();

        mecanumDrive.driveBackwards(18);
        checkForStop();

        // TODO: Latch onto foundation

        mecanumDrive.driveForwards(18);
        checkForStop();

        // TODO: Unlatch
    }

    public boolean isDanCute() {
        return true;
        // Emily M chen wrote this on 11/20/2019 at 6:11 PM
    }
}

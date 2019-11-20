package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousMaster;

@Autonomous(name = "Blue Move Foundation (Tile 4)", group = "blue")
public class MoveFoundationTile4 extends AutonomousMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        mecanumDrive.driveForwards(15);
        checkForStop();
        mecanumDrive.rotateCounterClockwise(90);
        checkForStop();
        mecanumDrive.driveForwards(24);
        checkForStop();
        mecanumDrive.rotateCounterClockwise(90);
        checkForStop();
        mecanumDrive.driveBackwards(15);
        checkForStop();
        //TODO: hook onto board
        mecanumDrive.driveForwards(32);
        checkForStop();
        //TODO: unhook
        mecanumDrive.strafeLeft(24);

    }
}

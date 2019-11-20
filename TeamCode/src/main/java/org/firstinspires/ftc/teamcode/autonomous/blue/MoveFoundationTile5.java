package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousMaster;

@Autonomous(name = "Blue Move Foundation (Tile 5)", group = "Blue")
public class MoveFoundationTile5 extends AutonomousMaster {
    private String variable = "Hello";

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(); // Initializes and waits for start with pings

        mecanumDrive.driveForwards(15);
        checkForStop();
        mecanumDrive.rotateClockwise(180);
        checkForStop();
        mecanumDrive.driveBackwards(15);
        checkForStop();

        // TODO: Latch on to foundation

        mecanumDrive.driveForwards(32);

        // TODO: Unlatch foundation

        mecanumDrive.strafeLeft(16);
    }
}

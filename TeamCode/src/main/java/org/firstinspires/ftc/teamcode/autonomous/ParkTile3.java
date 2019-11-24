package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park (Tile 3)", group = "A")
public class ParkTile3 extends AutonomousMaster {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(); // Initializes and waits for start with pings
        telemetry.addLine("Running...");
        telemetry.update();

        mecanumDrive.setMovementPower(-0.5);
        sleep(375);
        mecanumDrive.stop();
    }
}

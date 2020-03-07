package org.firstinspires.ftc.teamcode.autonomous.alliance_insignificant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousMaster;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousMasterRoadrunner;

@Autonomous(name = "Park (Tiles 3 or 4)")
public class ParkTile3 extends AutonomousMasterRoadrunner {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(); // Initializes and waits for start with pings
        telemetry.addLine("Running...");
        telemetry.update();

        checkForStop();
        drive(bot -> bot.forward(16));
    }
}
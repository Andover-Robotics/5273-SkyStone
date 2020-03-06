package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousMasterRoadrunner;
import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;
import org.firstinspires.ftc.teamcode.vision.SkystoneLocation;

public class ParkBridgeTile3 extends AutonomousMasterRoadrunner {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(); // Initializes and configures

        checkForStop();

        // Right side of bot starts from the left edge of tile 3
        double startingPositionX = -24 + GlobalConfig.BOT_WIDTH_IN / 2;
        double startingPositionY = -72 + GlobalConfig.BOT_LENGTH_IN / 2;

        double bridgePositionY = -24 - GlobalConfig.BOT_LENGTH_IN / 2;

        driveBase.setPoseEstimate(new Pose2d(new Vector2d(startingPositionX, startingPositionY), 0));
        Pose2d underBridge = new Pose2d(new Vector2d(0, bridgePositionY), 0);

        drive(bot -> bot.splineTo(underBridge));
    }
}

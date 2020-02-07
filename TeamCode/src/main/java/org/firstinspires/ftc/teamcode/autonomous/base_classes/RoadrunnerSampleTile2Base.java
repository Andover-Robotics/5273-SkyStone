package org.firstinspires.ftc.teamcode.autonomous.base_classes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;

public abstract class RoadrunnerSampleTile2Base extends AutonomousBaseRoadrunner {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(); // Initializes and configures

        while (lastStoneLocation == null) {
            lastStoneLocation = scanForStone();
        }

        // Start at tile 2, facing the building zone, against the wall
        driveBase.setPoseEstimate(new Pose2d(new Vector2d(-48 + GlobalConfig.BOT_LENGTH_IN / 2, 72 - GlobalConfig.BOT_WIDTH_IN / 2), 0));

        skystoneDetector.setFlashLight(false);
        skystoneDetector.stop();
        drive(bot -> bot.strafeRight(34.5));

        double distanceForward = 3.25 * 24;
        switch (lastStoneLocation) {
            case MIDDLE:
                drive(bot -> bot.forward(8));
                distanceForward -= 8;
                break;
            case LEFT:
                drive(bot -> bot.forward(16));
                distanceForward -= 16;
                break;
        }


        if (currentAlliance == RobotAlliance.BLUE) {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        }

        sleep(1500);
        drive(bot -> bot.strafeLeft(11));

        final double FINAL_DISTANCE_FORWARD = distanceForward; // forward requires finals for some reason
        drive(bot -> bot.forward(FINAL_DISTANCE_FORWARD));

        drive(bot -> bot.strafeRight(12));

        if (currentAlliance == RobotAlliance.BLUE) {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);
            sleep(500);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        }

        sleep(250);

        drive(bot -> bot.strafeLeft(15));

        setLiftPower(0.65);
        driveBase.turnSync(-Math.PI / 2);
        holdLiftLocation();
        drive(bot -> bot.forward(13));

        // TODO: Translate encoder-based auto to roadrunner
    }

}

package org.firstinspires.ftc.teamcode.autonomous.base_classes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;
import org.firstinspires.ftc.teamcode.vision.SkystoneLocation;

public abstract class RoadrunnerSampleTile2Base extends AutonomousBaseRoadrunner {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(); // Initializes and configures

        while (lastStoneLocation == null) {
            lastStoneLocation = scanForStone();
        }

        // Start at tile 2, facing the building zone, against the wall
        double startingPositionX = -48 + GlobalConfig.BOT_LENGTH_IN / 2;

        driveBase.setPoseEstimate(new Pose2d(new Vector2d(-48 + GlobalConfig.BOT_LENGTH_IN / 2, 72 - GlobalConfig.BOT_WIDTH_IN / 2), 0));

        skystoneDetector.setFlashLight(false);
        skystoneDetector.stop();

        if (lastStoneLocation == SkystoneLocation.MIDDLE) {
            startingPositionX += 8;
        } else if (lastStoneLocation == SkystoneLocation.LEFT) {
            startingPositionX += 16;
        }

        final double endingPositionX = startingPositionX;

        drive(bot -> bot.splineTo(new Pose2d(new Vector2d(endingPositionX, 46 - GlobalConfig.BOT_WIDTH_IN / 2), 0)));

        if (currentAlliance == RobotAlliance.BLUE) {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        }

        sleep(1500);
        drive(bot -> bot.splineTo(new Pose2d(new Vector2d(0, 48 - GlobalConfig.BOT_WIDTH_IN / 2), 0)).splineTo(new Pose2d(new Vector2d(24, 40 - GlobalConfig.BOT_WIDTH_IN / 2), 0)));

        //drive(bot -> bot.forward(2.25 * 24));

        drive(bot -> bot.strafeRight(8));

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

        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_DOWN);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_DOWN);

        setLiftPower(0.0075);

        drive(bot -> bot.splineTo(new Pose2d(new Vector2d(24 - GlobalConfig.BOT_LENGTH_IN / 2, 48 - GlobalConfig.BOT_WIDTH_IN / 2), 0)).reverse());
        setLiftPower(0);

        // TODO: Translate encoder-based auto to roadrunner
    }

}

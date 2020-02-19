package org.firstinspires.ftc.teamcode.autonomous.base_classes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;
import org.firstinspires.ftc.teamcode.vision.SkystoneLocation;
import org.opencv.core.Mat;

public abstract class RoadrunnerSampleTile2Base extends AutonomousBaseRoadrunner {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(); // Initializes and configures

        int allianceDistanceMultiplier = currentAlliance == RobotAlliance.BLUE ? 1 : -1;

        while (lastStoneLocation == null) {
            lastStoneLocation = scanForStone();
        }

        // Start at tile 2, facing the building zone, against the wall
        double startingPositionX = -48 + GlobalConfig.BOT_LENGTH_IN / 2;

        driveBase.setPoseEstimate(new Pose2d(new Vector2d(-48 + GlobalConfig.BOT_LENGTH_IN / 2, allianceDistanceMultiplier * (72 - GlobalConfig.BOT_WIDTH_IN / 2)), 0));

        skystoneDetector.setFlashLight(false);
        skystoneDetector.stop();

        if (lastStoneLocation == SkystoneLocation.MIDDLE) {
            startingPositionX += 8;
        } else if (currentAlliance == RobotAlliance.BLUE && lastStoneLocation == SkystoneLocation.LEFT || currentAlliance == RobotAlliance.RED && lastStoneLocation == SkystoneLocation.RIGHT) {
            startingPositionX += 16;
        }

        final double endingPositionX = startingPositionX;

        drive(bot -> bot.splineTo(new Pose2d(new Vector2d(endingPositionX,  allianceDistanceMultiplier * (45.25 - GlobalConfig.BOT_WIDTH_IN / 2)), 0)));

        if (currentAlliance == RobotAlliance.BLUE) {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        } else {
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN);
            sleep(500);
            sideClawFingerLeft.setPosition(GlobalConfig.SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
        }

        sleep(1000);
        drive(bot -> bot.splineTo(new Pose2d(new Vector2d(0, allianceDistanceMultiplier * (48 - GlobalConfig.BOT_WIDTH_IN / 2)), 0)).splineTo(new Pose2d(new Vector2d(35, allianceDistanceMultiplier * (41.875  - GlobalConfig.BOT_WIDTH_IN / 2)), 0)));

        //drive(bot -> bot.forward(2.25 * 24));

        //drive(bot -> bot.strafeRight(8));

        if (currentAlliance == RobotAlliance.BLUE) {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);
            sleep(500);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        } else {
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerLeft.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);
            sleep(500);
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
        }

        sleep(250);

        //drive(bot -> bot.splineTo(new Pose2d(new Vector2d(48,allianceDistanceMultiplier * (30+GlobalConfig.BOT_WIDTH_IN / 2)),0)));
        //drive(bot -> bot.strafeLeft(7));
        //drive(bot -> bot.forward(10));

        // s n a t ch foundation
        /*
        driveBase.turnSync(allianceDistanceMultiplier * (-Math.PI) * 1.05 / 2);
        setLiftPower(0.6);
        sleep(750);
        holdLiftLocation();
        drive(bot -> bot.forward(8.5));

        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_DOWN);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_DOWN);

        setLiftPower(0.0075);
        sleep(750);*/

        // put near bridge
        /*driveBase.turnSync(allianceDistanceMultiplier * Math.PI/2);
        drive(bot -> bot.back(16));
        driveBase.turnSync(allianceDistanceMultiplier * Math.PI/2);
        drive(bot -> bot.back(8));*/

        // put in building zone
        /*drive(bot -> bot.back(32));
        driveBase.turnSync(allianceDistanceMultiplier * 1.3*Math.PI/2);

        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_UP);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_UP);*/

        //drive(bot -> bot.reverse().splineTo(new Pose2d(new Vector2d(endingPositionX - 16, allianceDistanceMultiplier * 46 - GlobalConfig.BOT_WIDTH_IN / 2)), 0)));
        drive(bot -> bot.reverse().splineTo(new Pose2d(new Vector2d(0, allianceDistanceMultiplier * (48 - GlobalConfig.BOT_WIDTH_IN / 2)), 0)).splineTo(new Pose2d(new Vector2d(endingPositionX - 16, allianceDistanceMultiplier * (43 - GlobalConfig.BOT_WIDTH_IN / 2)), 0)));


        if (currentAlliance == RobotAlliance.BLUE) {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        } else {
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN);
            sleep(500);
            sideClawFingerLeft.setPosition(GlobalConfig.SIDE_CLAW_FINGER_CLOSE);
            sleep(1000);
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
        }

        sleep(1000);
        drive(bot -> bot.splineTo(new Pose2d(new Vector2d(0, allianceDistanceMultiplier * (48 - GlobalConfig.BOT_WIDTH_IN / 2)), 0)).splineTo(new Pose2d(new Vector2d(40, allianceDistanceMultiplier * (41  - GlobalConfig.BOT_WIDTH_IN / 2)), 0)));

        if (currentAlliance == RobotAlliance.BLUE) {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerRight.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);
            sleep(500);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        } else {
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN);
            sleep(250);
            sideClawFingerLeft.setPosition(GlobalConfig.SIDE_CLAW_FINGER_OPEN);
            sleep(500);
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
        }

        //drive(bot -> bot.splineTo(new Pose2d(new Vector2d(12, 48 - GlobalConfig.BOT_WIDTH_IN / 2), 0)).reverse());
        //drive(bot -> bot.back(32));
        //driveBase.turnSync(1.3*Math.PI/2);

        setLiftPower(0);

        // TODO: Translate encoder-based auto to roadrunner
    }

}

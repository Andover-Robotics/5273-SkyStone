package org.firstinspires.ftc.teamcode.autonomous.base_classes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.autonomous.RobotAlliance;
import org.firstinspires.ftc.teamcode.vision.SkystoneLocation;

public abstract class RoadrunnerSampleTile2Base extends AutonomousBaseRoadrunner {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode(); // Initializes and configures

        checkForStop();

        int allianceDistanceMultiplier = currentAlliance == RobotAlliance.BLUE ? 1 : -1;

        while (lastStoneLocation == null) {
            lastStoneLocation = scanForStone();
        }

        if (currentAlliance == RobotAlliance.BLUE)
            lastStoneLocation = swapSkystoneLocation(lastStoneLocation);

        // Start at tile 2, facing the building zone, against the wall
        double startingPositionX = -47.5 + GlobalConfig.BOT_LENGTH_IN / 2;

        driveBase.setPoseEstimate(new Pose2d(new Vector2d(startingPositionX, allianceDistanceMultiplier * (72 - GlobalConfig.BOT_WIDTH_IN / 2)), 0));

        skystoneDetector.setFlashLight(false);
        skystoneDetector.stop();

        //drive(bot -> bot.strafeRight(allianceDistanceMultiplier * (42.5 - GlobalConfig.BOT_WIDTH_IN / 2)));

        final double firstSkystoneX;

        if(lastStoneLocation == SkystoneLocation.LEFT) {
            firstSkystoneX = -75 + GlobalConfig.BOT_LENGTH_IN / 2;
        } else {
            firstSkystoneX = -70 + (lastStoneLocation.getNumericalValue() * 8 + 4 + GlobalConfig.BOT_LENGTH_IN / 2 - GlobalConfig.BOT_CLAW_FROM_BACK) - 3; //(lastStoneLocation == SkystoneLocation.LEFT ? 2 : 4)) + GlobalConfig.BOT_LENGTH_IN / 2;
        }

        Pose2d underBridge = new Pose2d(new Vector2d(6, allianceDistanceMultiplier * (46.5 - GlobalConfig.BOT_WIDTH_IN / 2)), 0);
        double foundationPlaceY = (35 - GlobalConfig.BOT_WIDTH_IN / 2);

        final double finalSkystoneX = firstSkystoneX + 36;

        checkForStop();

        // DRIVE TO FIRST SKYSTONE
        drive(bot -> bot.strafeTo(new Vector2d(firstSkystoneX, allianceDistanceMultiplier * (27 + GlobalConfig.BOT_WIDTH_IN / 2))));
//        drive(bot -> bot.splineTo(new Pose2d(new Vector2d(firstSkystoneX, allianceDistanceMultiplier * (26.5 + GlobalConfig.BOT_WIDTH_IN / 2)), 0), new ConstantInterpolator(0)));

        grabStone(currentAlliance, 750);

        drive(bot -> bot.strafeLeft(3 * allianceDistanceMultiplier));

        // DRIVE TO FOUNDATION TO DEPOSIT FIRST STONE
        drive(bot -> bot.splineTo(underBridge).splineTo(new Pose2d(new Vector2d(44 + GlobalConfig.BOT_LENGTH_IN/2 - GlobalConfig.BOT_CLAW_FROM_BACK, allianceDistanceMultiplier * foundationPlaceY), 0)));

        throwStone(currentAlliance, 250);

        final double distanceToAlignWithStone;

        /*switch (lastStoneLocation) {
            case RIGHT:
                distanceToAlignWithStone = 12.5;
                break;
            case MIDDLE:
                distanceToAlignWithStone = 3.5;
                break;
            default:
                distanceToAlignWithStone = -4;
                break;
        }*/


        switch(currentAlliance) {
            case RED:
                drive(bot -> bot.strafeRight(4));
                break;
            case BLUE:
                drive(bot -> bot.strafeLeft(4));
                break;
        }

        // DRIVE TO SECOND STONE`
        drive(bot -> bot.reverse().splineTo(underBridge).splineTo(new Pose2d(new Vector2d(finalSkystoneX - 0.5, allianceDistanceMultiplier * (29.75 + GlobalConfig.BOT_WIDTH_IN / 2)), 0)));

        /*if(lastStoneLocation == SkystoneLocation.LEFT){
            driveBase.turnSync(allianceDistanceMultiplier * -Math.PI/8);
        }*/

        grabStone(currentAlliance, 750);

        /*if(lastStoneLocation == SkystoneLocation.LEFT){
            driveBase.turnSync(allianceDistanceMultiplier * Math.PI/8);
        }*/

        // DRIVE TO FOUNDATION TO DEPOSIT SECOND STONE
        drive(bot -> bot.splineTo(underBridge).splineTo(new Pose2d(new Vector2d(34 + GlobalConfig.BOT_LENGTH_IN/2 - GlobalConfig.BOT_CLAW_FROM_BACK, allianceDistanceMultiplier * foundationPlaceY), 0)));

        throwStone(currentAlliance, 250);

        // Close left finger to allow wire train to move when slides go up
        sideClawFingerLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_FINGER_CLOSE);

        drive(bot -> bot.forward(8));
        setLiftPower(.50);
        driveBase.turnSync(allianceDistanceMultiplier * - Math.PI / 2);
        holdLiftLocation();

        drive(bot -> bot.forward(4));

        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_DOWN);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_DOWN);

        sleep(500);

        //drive(bot -> bot.back(55));

        driveBase.turnSync(allianceDistanceMultiplier * Math.PI);

        //drive(bot -> bot.forward(8));

        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_UP);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_UP);

        drive(bot -> bot.strafeRight(40 * allianceDistanceMultiplier));
        setLiftPower(0.0025);
        drive(bot -> bot.splineTo(new Pose2d(new Vector2d(6, allianceDistanceMultiplier * (50 - GlobalConfig.BOT_WIDTH_IN / 2)), 0)));
        setLiftPower(0);
        switch(currentAlliance) {
            case RED:
                sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
                sideClawFingerRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_FINGER_OUT);
                break;
            case BLUE:
                sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN);
                sideClawFingerLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_FINGER_OUT);
                break;
        }
    }

}
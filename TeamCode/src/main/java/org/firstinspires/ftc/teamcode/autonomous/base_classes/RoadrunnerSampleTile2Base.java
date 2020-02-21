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

        checkForStop();

        int allianceDistanceMultiplier = currentAlliance == RobotAlliance.BLUE ? 1 : -1;

        while (lastStoneLocation == null) {
            lastStoneLocation = scanForStone();
        }

        if (currentAlliance == RobotAlliance.BLUE)
            lastStoneLocation = swapSkystoneLocation(lastStoneLocation);

        // Start at tile 2, facing the building zone, against the wall
        double startingPositionX = -48 + GlobalConfig.BOT_LENGTH_IN / 2;

        driveBase.setPoseEstimate(new Pose2d(new Vector2d(startingPositionX, allianceDistanceMultiplier * (72 - GlobalConfig.BOT_WIDTH_IN / 2)), 0));

        skystoneDetector.setFlashLight(false);
        skystoneDetector.stop();

        //drive(bot -> bot.strafeRight(allianceDistanceMultiplier * (42.5 - GlobalConfig.BOT_WIDTH_IN / 2)));

        final double distanceToAlignWithStone;

        switch (lastStoneLocation) {
            case RIGHT:
                distanceToAlignWithStone = 12.5;
                break;
            case MIDDLE:
                distanceToAlignWithStone = 3.5;
                break;
            default:
                distanceToAlignWithStone = -4;
                break;
        }

        checkForStop();

        drive(bot -> bot.strafeTo(new Vector2d(startingPositionX + distanceToAlignWithStone, allianceDistanceMultiplier * (18.5 + GlobalConfig.BOT_WIDTH_IN / 2))));

        grabStone(currentAlliance, 500);

        Pose2d underBridge = new Pose2d(new Vector2d(0, allianceDistanceMultiplier * (46 - GlobalConfig.BOT_WIDTH_IN / 2)), 0);
        double foundationPlaceY = allianceDistanceMultiplier * (44 - GlobalConfig.BOT_WIDTH_IN / 2);

        drive(bot -> bot.splineTo(underBridge).splineTo(new Pose2d(new Vector2d(37, foundationPlaceY), 0)));

        placeStone(currentAlliance, 250);

        final double finalSkystoneX = -72 + (lastStoneLocation.getNumericalValue() * 8 + (lastStoneLocation == SkystoneLocation.LEFT ? 2 : 4)) + GlobalConfig.BOT_LENGTH_IN / 2;

        drive(bot -> bot.reverse().splineTo(new Pose2d(new Vector2d(0, allianceDistanceMultiplier * (49 - GlobalConfig.BOT_WIDTH_IN / 2)), 0)).splineTo(new Pose2d(new Vector2d(finalSkystoneX, allianceDistanceMultiplier * (25 + GlobalConfig.BOT_WIDTH_IN / 2)), 0)));

        grabStone(currentAlliance, 500);

        drive(bot -> bot.splineTo(underBridge).splineTo(new Pose2d(new Vector2d(23, foundationPlaceY * 1.125), 0)).splineTo(new Pose2d(new Vector2d(45.5, foundationPlaceY - 2 * allianceDistanceMultiplier), 0)));

        placeStone(currentAlliance, 250);

        drive(bot -> bot.forward(4));

        setLiftPower(0.65);
        driveBase.turnSync(allianceDistanceMultiplier * -Math.PI / 2);
        holdLiftLocation();

        drive(bot -> bot.forward(6));

        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_DOWN);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_DOWN);

        sleep(500);

        drive(bot -> bot.back(30));

        driveBase.turnSync(allianceDistanceMultiplier * Math.PI);

        drive(bot -> bot.forward(8));

        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_UP);
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_UP);

        setLiftPower(0.0075);
        drive(bot -> bot.reverse().splineTo(underBridge));
        setLiftPower(0);
    }

}

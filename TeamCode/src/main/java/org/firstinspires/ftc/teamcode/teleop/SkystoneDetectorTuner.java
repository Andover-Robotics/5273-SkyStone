package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.SkystoneDetector;

/**
 * Allows the parameters of a SkystoneDetector to be tuned with the controller.
 * This is intended to be used in the following way:
 * First, position the robot in the location from which it will be scanning for skystones
 * Next, INIT this program on the driver station (DO NOT START)
 * Press the 3 dot menu on the driver station and hit "Camera Stream"
 * The camera of the robot controller should be visible on your driver station
 * <p>
 * Adjust the parameters using the gamepad:
 * Pressing up and down on the dpad will adjust the stoneBaseYCoordinate by larger intervals
 * Pressing Y and A will adjust the stoneBaseYCoordinate by smaller intervals
 * Pressing left and right on the dpad will adjust the stoneWidth by larger intervals
 * Pressing X and B will adjust the stoneWidth by smaller intervals
 * Pressing the left and right triggers will adjust the stoneHeight by larger intervals
 * Pressing the left and right bumpers will adjust the stoneHeight by smaller intervals
 * <p>
 * Tap the camera stream on your driver station to refresh it and view the new changes
 * Continue tuning until the large rectangles roughly fit around the 3 stones
 * Look at the telemetry to see the values of the 3 parameters and the perceived location of the skystone
 * When everything is adjusted, record the final values of each. Use them to modify the constructor of the SkystoneDetector in your autonomous OpMode.
 */
@TeleOp(name = "Skystone Detector Tuner", group = "AB")
public class SkystoneDetectorTuner extends LinearOpMode {

    private SkystoneDetector detector;
    private int stoneBaseYCoordinate = 0, stoneWidth = 200, stoneHeight = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        detector = new SkystoneDetector(hardwareMap, stoneBaseYCoordinate, stoneWidth, stoneHeight);
        detector.start();

        while (!isStarted()) {
            if (isStopRequested())
                break;


            // stoneBaseYCoordinate
            if (gamepad1.dpad_up)
                stoneBaseYCoordinate += 5;
            else if (gamepad1.dpad_down)
                stoneBaseYCoordinate -= 5;
            else if (gamepad1.y)
                stoneBaseYCoordinate += 1;
            else if (gamepad1.a)
                stoneBaseYCoordinate -= 1;


            // stoneWidth
            if (gamepad1.dpad_left)
                stoneWidth -= 5;
            else if (gamepad1.dpad_right)
                stoneWidth += 5;
            else if (gamepad1.x)
                stoneWidth -= 1;
            else if (gamepad1.b)
                stoneWidth += 1;

            // stoneHeight
            if (gamepad1.right_trigger > 0.1)
                stoneHeight += 5;
            else if (gamepad1.left_trigger > 0.1)
                stoneHeight -= 5;
            else if (gamepad1.right_bumper)
                stoneHeight += 1;
            else if (gamepad1.left_bumper)
                stoneHeight -= 1;

            // Update detector values
            detector.setStoneBaseYCoordinate(stoneBaseYCoordinate);
            detector.setStoneWidth(stoneWidth);
            detector.setStoneHeight(stoneHeight);

            // Log current values
            telemetry.addData("stoneBaseYCoordinate", stoneBaseYCoordinate);
            telemetry.addData("stoneWidth", stoneWidth);
            telemetry.addData("stoneHeight", stoneHeight);
            telemetry.addData("Perceived skystone location", detector.getSkystoneLocation());
            telemetry.update();
        }

        detector.stop();
    }
}

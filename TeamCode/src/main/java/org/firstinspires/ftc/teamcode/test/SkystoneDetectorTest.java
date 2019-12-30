package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.SkystoneDetector;

@TeleOp(name = "Skystone Detector Test", group = "Test")
public class SkystoneDetectorTest extends LinearOpMode {
    private SkystoneDetector skystoneDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        skystoneDetector = new SkystoneDetector(hardwareMap, 280, 230, 130);
        skystoneDetector.start();

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addLine(skystoneDetector.getSkystoneLocation().toString());
            telemetry.update();
        }

        skystoneDetector.stop();
    }
}

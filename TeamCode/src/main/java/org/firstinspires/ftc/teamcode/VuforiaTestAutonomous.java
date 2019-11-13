package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.vision.VuforiaManager;

@Autonomous(name = "Vuforia Test", group = "Test")
public class VuforiaTestAutonomous extends LinearOpMode {

    private VuforiaManager vuforiaManager;

    private MecanumDrive mecanumDrive;
    private DcMotor motorFL, motorFR, motorBL, motorBR;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("Status", "READY");
        telemetry.update();

        waitForStart();

        vuforiaManager.activate();

        telemetry.addData("Status", "RUNNING");
        telemetry.update();


        OpenGLMatrix targetLocation = vuforiaManager.getTrackableLocation("Blue Perimeter 1");
        double targetYCoordinate = (double) targetLocation.getData()[1];
        mecanumDrive.setMovementPower(0.2);

        double robotY = vuforiaManager.getRobotY();

        while (robotY < targetYCoordinate) {
            telemetry.addData("robotX", vuforiaManager.getRobotX());
            telemetry.addData("robotY", robotY);
            telemetry.update();

            robotY = vuforiaManager.getRobotY();

            checkForStop();
            idle();
        }

        mecanumDrive.setMovementPower(0);

        sleep(500);

        mecanumDrive.setMovementPower(0.2);

        while (robotY < vuforiaManager.getTrackableLocation("Blue Perimeter 2").getData()[1]) {
            telemetry.addData("robotX", vuforiaManager.getRobotX());
            telemetry.addData("robotY", robotY);
            telemetry.update();

            robotY = vuforiaManager.getRobotY();

            checkForStop();
            idle();
        }

        mecanumDrive.setMovementPower(0);


        vuforiaManager.shutdown();
        telemetry.addData("robotX", vuforiaManager.getRobotX());
        telemetry.addData("robotY", vuforiaManager.getRobotY());
        telemetry.update();

        sleep(5000);
        stop();
    }

    private void initialize() throws InterruptedException {
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, 63, 5200);

        vuforiaManager = new VuforiaManager(telemetry, hardwareMap);
    }

    private void checkForStop() throws InterruptedException {
        if (isStopRequested())
            throw new InterruptedException();
    }
}

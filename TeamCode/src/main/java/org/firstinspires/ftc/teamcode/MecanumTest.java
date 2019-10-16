package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@Autonomous(name = "Mecanum Test", group = "Test")
public class MecanumTest extends LinearOpMode {

    private MecanumDrive mecanumDrive;
    private DcMotor motorFL, motorFR, motorBL, motorBR;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        telemetry.addData("Status", "READY");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "RUNNING");
        telemetry.update();

        mecanumDrive.driveForwards(24);
        mecanumDrive.strafeRight(24);
        mecanumDrive.driveBackwards(24);
        mecanumDrive.strafeLeft(24);

        stop();
    }

    private void initialize() {
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setDirection(Direction.REVERSE);
        motorBL.setDirection(Direction.REVERSE);

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, 63, 5200);
        mecanumDrive.setDefaultDrivePower(0.5);
    }
}
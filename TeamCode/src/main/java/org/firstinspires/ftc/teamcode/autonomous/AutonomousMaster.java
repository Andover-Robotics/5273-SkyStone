package org.firstinspires.ftc.teamcode.autonomous;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class AutonomousMaster extends LinearOpMode {

    protected MecanumDrive mecanumDrive;
    private DcMotor motorFL, motorFR, motorBL, motorBR;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStartWithPings();

        telemetry.addData("Status", "STARTING");
        telemetry.update();
    }

    protected void checkForStop() throws InterruptedException {
        if (isStopRequested())
            throw new InterruptedException();
    }

    protected void initialize() {
        telemetry.addData("Status", "INITIALIZING");
        telemetry.update();

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, GlobalConfig.TICKS_PER_INCH, GlobalConfig.TICKS_PER_360);
    }

    // Waits for the OpMode to be run while sending messages between the phones
    // This avoids timeouts and resolves a common bug that often results in loss of connection
    private void waitForStartWithPings() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "WAITING");
            telemetry.addData("Time", System.currentTimeMillis());
            telemetry.update();
        }

        telemetry.update();
    }
}


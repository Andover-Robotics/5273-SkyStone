package org.firstinspires.ftc.teamcode.autonomous;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.vision.SkystoneDetector;

public class AutonomousMaster extends LinearOpMode {

    protected MecanumDrive mecanumDrive;
    protected SkystoneDetector skystoneDetector;
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

        DcMotor[] motors = {motorFL, motorFR, motorBL, motorBR};

        // Adjust the tolerances and PID coefficients of motors to prevent micro-adjustments after movement
        for(DcMotor motor: motors) {
            DcMotorEx motorEX = (DcMotorEx) motor;
            motorEX.setTargetPositionTolerance((int)(0.45 * GlobalConfig.TICKS_PER_INCH + 0.5));
            PIDFCoefficients coefficients = motorEX.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            motorEX.setVelocityPIDFCoefficients(coefficients.p + 0.1, coefficients.i + 0.5, coefficients.d, coefficients.f + 0.2);
        }

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, GlobalConfig.TICKS_PER_INCH, GlobalConfig.TICKS_PER_360);
        mecanumDrive.setDefaultDrivePower(0.25);

        skystoneDetector = new SkystoneDetector(hardwareMap, 280, 230, 130);
        skystoneDetector.start();
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


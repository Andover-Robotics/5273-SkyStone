package org.firstinspires.ftc.teamcode;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.*;


@Autonomous(name = "KeVin Test", group = "Other")
public class KeVinTest extends LinearOpMode {
    private MecanumDrive mecanumDrive;
    private DcMotor motorFL, motorFR, motorBL, motorBR;

    public void runOpMode() {
        initialize();
        waitForStart();
        telemetry.addData("Status","RUNNING");
        telemetry.update();

        Random number = new Random();
        for (int n = 0; n < 10; n++) {
            int var = number.nextInt(4);
            if (var == 0) {
                mecanumDrive.driveForwards(10, 0.5);
                telemetry.addData("Direction","FORWARDS");
                telemetry.update();
            }
            else if (var == 1) {
                mecanumDrive.strafeRight(10,0.5);
                telemetry.addData("Direction","RIGHT");
                telemetry.update();
            }
            else if (var == 2) {
                mecanumDrive.driveBackwards(10,0.5);
                telemetry.addData("Direction","BACKWARDS");
                telemetry.update();
            }
            else {
                mecanumDrive.strafeLeft(10, 0.5);
                telemetry.addData("Direction","LEFT");
                telemetry.update();
            }
            telemetry.addData("test",n);
            telemetry.update();
            sleep(1000);
        }
        stop();
    }
    private void initialize() {
        telemetry.addData("KEVINBIGGAY?","TRUE");
        telemetry.update();
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);



        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, 63, 5200);
        mecanumDrive.setDefaultDrivePower(0.25);
    }
}

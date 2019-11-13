package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Motor Tester", group = "Test")
public class MotorTester extends LinearOpMode {

    private DcMotor motorFL, motorFR, motorBL, motorBR;

    @Override
    public void runOpMode() throws InterruptedException {
        initMotors();

        waitForStart();

        telemetry.addData("Now Testing", "motorFR");
        telemetry.update();

        motorFR.setPower(1);
        sleep(1000);
        motorFR.setPower(0);

        telemetry.addData("Now Testing", "motorFL");
        telemetry.update();

        motorFL.setPower(1);
        sleep(1000);
        motorFL.setPower(0);

        telemetry.addData("Now Testing", "motorBR");
        telemetry.update();


        motorBR.setPower(1);
        sleep(1000);
        motorBR.setPower(0);

        telemetry.addData("Now Testing", "motorBL");
        telemetry.update();

        motorBL.setPower(1);
        sleep(1000);
        motorBL.setPower(0);

    }

    public void initMotors() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
    }
}

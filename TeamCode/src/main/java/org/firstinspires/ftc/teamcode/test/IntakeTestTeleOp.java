package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Intake Test TeleOp", group = "Tesr")
public class IntakeTestTeleOp extends OpMode {

    private CRServo left, right;

    @Override
    public void init() {
        left = hardwareMap.crservo.get("intakeLeft");
        right = hardwareMap.crservo.get("intakeRight");

        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double power = 0;

        if (gamepad1.left_trigger > 0) {
            power = -gamepad1.left_trigger;
        }


        if (gamepad1.right_trigger > 0) {
            power = gamepad1.right_trigger;
        }

        left.setPower(power);
        right.setPower(power);
    }
}

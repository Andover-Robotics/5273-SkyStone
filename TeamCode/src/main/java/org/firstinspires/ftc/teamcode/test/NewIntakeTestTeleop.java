package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "New Intake Test TeleOp", group = "Test")
public class NewIntakeTestTeleop extends OpMode {

    private DcMotor left, right;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            left.setPower(1);
            right.setPower(-1);
        } else {
            left.setPower(0);
            right.setPower(0);
        }

        if(gamepad1.b) {
            left.setPower(-1);
            right.setPower(1);
        } else {
            left.setPower(0);
            right.setPower(0);
        }

        if(gamepad1.x) {
            left.setPower(1);
            right.setPower(1);
        } else {
            left.setPower(0);
            right.setPower(0);
        }

        if(gamepad1.y) {
            left.setPower(-1);
            right.setPower(-1);
        } else {
            left.setPower(0);
            right.setPower(0);
        }
    }
}

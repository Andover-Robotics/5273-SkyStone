package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeTester extends OpMode {

    private CRServo intakeServoLeft, intakeServoRight;

    @Override
    public void init() {
        intakeServoLeft = hardwareMap.crservo.get("intakeLeft");
        intakeServoRight = hardwareMap.crservo.get("intakeRight");
    }

    @Override
    public void loop() {
        double intakePower = 0;
        if (gamepad1.left_bumper)
            intakePower = 1;
        else if (gamepad1.right_bumper)
            intakePower = -1;

        intakeServoLeft.setPower(intakePower);
        intakeServoRight.setPower(-intakePower);
    }
}

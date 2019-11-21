package org.firstinspires.ftc.teamcode.test;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp", group = "Test")
public class MecanumTestTeleOp extends OpMode {

    private MecanumDrive mecanumDrive;
    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private Servo foundationServo;

    @Override
    public void init() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        foundationServo = hardwareMap.servo.get("foundationMover");
        foundationServo.setPosition(0);

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, 63, 5200);
    }

    @Override
    public void loop() {
        Coordinate strafe = Coordinate.fromXY(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        mecanumDrive.setStrafe(strafe);
        mecanumDrive.setRotationPower(-gamepad1.right_stick_x);

        if (gamepad1.left_bumper)
            foundationServo.setPosition(0);
        else if(gamepad1.right_bumper)
            foundationServo.setPosition(0.69);
    }
}

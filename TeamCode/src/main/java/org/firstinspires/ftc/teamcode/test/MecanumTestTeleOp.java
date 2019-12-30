package org.firstinspires.ftc.teamcode.test;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum Drive Test TeleOp", group = "Test")
public class  MecanumTestTeleOp extends OpMode {

    private MecanumDrive mecanumDrive;
    private DcMotor motorFL, motorFR, motorBL, motorBR;

    @Override
    public void init() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, 63, 5200);
    }

    @Override
    public void loop() {
        Coordinate strafe = Coordinate.fromXY(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        mecanumDrive.setStrafe(strafe);
        mecanumDrive.setRotationPower(-gamepad1.right_stick_x);
    }
}

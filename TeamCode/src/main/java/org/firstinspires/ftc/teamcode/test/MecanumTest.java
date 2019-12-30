package org.firstinspires.ftc.teamcode.test;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousMaster;

@Autonomous(name = "Mecanum Test", group = "Test")
public class MecanumTest extends AutonomousMaster {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        for (int i = 0; i < 4; i++) {
            mecanumDrive.driveForwards(24);
            mecanumDrive.rotateClockwise(90);
        }

        stop();
    }

}

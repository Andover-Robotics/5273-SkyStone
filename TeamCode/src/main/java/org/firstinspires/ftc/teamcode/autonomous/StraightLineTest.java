package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GlobalConfig;

import java.util.Arrays;
import java.util.Comparator;

@Autonomous(name = "Straight Line Test", group = "Test")
public class StraightLineTest extends AutonomousMaster {
    private DcMotor motorFL, motorFR, motorBL, motorBR;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        mecanumDrive.strafeRight(48, 0.5);

//        motorFL.setTargetPosition(GlobalConfig.TICKS_PER_INCH * 60);
//        motorFR.setTargetPosition(GlobalConfig.TICKS_PER_INCH * 60);
//        motorBL.setTargetPosition(GlobalConfig.TICKS_PER_INCH * 60);
//        motorBR.setTargetPosition(GlobalConfig.TICKS_PER_INCH * 60);
//
//        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        motorFL.setPower(0.5);
//        motorFR.setPower(0.5);
//        motorBL.setPower(0.5);
//        motorBR.setPower(0.5);
//
//        while (motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy()) {
//            telemetry.addData("motorFL", motorFL.getCurrentPosition());
//            telemetry.addData("motorFR", motorFR.getCurrentPosition());
//            telemetry.addData("motorBL", motorBL.getCurrentPosition());
//            telemetry.addData("motorBR", motorBR.getCurrentPosition());
//
//            Object[][] motorValues = {
//                    {"motorFL", motorFL.getCurrentPosition()},
//                    {"motorFR", motorFR.getCurrentPosition()},
//                    {"motorBL", motorBL.getCurrentPosition()},
//                    {"motorBR", motorBR.getCurrentPosition()}
//            };
//
//
//            Arrays.sort(motorValues, new Comparator<Object[]>() {
//                @Override
//                public int compare(Object[] objects, Object[] t1) {
//                    return (int) objects[1] - (int) t1[1];
//                }
//            });
//
//            String out = "";
//
//            for (Object[] motorData : motorValues)
//                out += (String) motorData[0] + ", ";
//
//            telemetry.addLine(out);
//
//            telemetry.update();
//        }

        stop();
    }
}

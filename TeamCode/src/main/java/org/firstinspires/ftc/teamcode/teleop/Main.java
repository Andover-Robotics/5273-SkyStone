package org.firstinspires.ftc.teamcode.teleop;

import com.andoverrobotics.core.drivetrain.MecanumDrive;
import com.andoverrobotics.core.utilities.Coordinate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

@TeleOp(name = "Main TeleOp", group = "AA") // Group is A to ensure this is at the top of the list
public class Main extends OpMode {
    private Servo foundationServoLeft, foundationServoRight;
    private MecanumDrive mecanumDrive;
    private CRServo intakeServoLeft, intakeServoRight;
    private DcMotor motorFL, motorFR, motorBL, motorBR, motorSlideLeft, motorSlideRight;
    private final double SLOW_MODE = 0.3;

    boolean isFalling = false;

    private Future<?> moveLiftMotor;
    private ExecutorService asyncExecutor = Executors.newSingleThreadExecutor();

    private Runnable moveLift = new Runnable() {
        @Override
        public void run() {
            try {
                int position = motorSlideRight.getCurrentPosition();
                boolean isMovingDown = gamepad2.b;

                if (position == 0 && !isMovingDown)
                    moveSlidesToStage(1);

                checkForInterrupt();
            } catch (InterruptedException e) {
            }
        }

        private void moveSlidesToStage(int stage) throws InterruptedException {
            double radiansPerStage = (GlobalConfig.LIFE_STAGE_HEIGHT_IN / 3) / (GlobalConfig.LIFT_PULLEY_RADIUS_MM / GlobalConfig.MM_PER_INCH);
            double ticksPerRadian = GlobalConfig.TICKS_PER_360 * 180 / Math.PI;
            double ticksPerStage = ticksPerRadian * radiansPerStage;

            double finalTicks = ticksPerStage * stage;
            int finalTicksInt = (int) Math.round(finalTicks);

            motorSlideLeft.setTargetPosition(finalTicksInt);
            motorSlideRight.setTargetPosition(finalTicksInt);

            motorSlideLeft.setPower(SLOW_MODE);
            motorSlideRight.setPower(SLOW_MODE);

            while (motorSlideLeft.isBusy() || motorSlideRight.isBusy()) {
                telemetry.addData("Left Position", motorSlideLeft.getCurrentPosition());
                telemetry.addData("Left Target", motorSlideLeft.getTargetPosition());
                telemetry.addData("Right Position", motorSlideRight.getCurrentPosition());
                telemetry.addData("Right Target", motorSlideRight.getTargetPosition());
                telemetry.update();
                checkForInterrupt();
            }

            motorSlideLeft.setPower(0);
            motorSlideRight.setPower(0);
        }
    };

    @Override
    public void init() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSlideLeft = hardwareMap.dcMotor.get("liftLeft");
        motorSlideRight = hardwareMap.dcMotor.get("liftRight");

        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeServoLeft = hardwareMap.crservo.get("intakeLeft");
        intakeServoRight = hardwareMap.crservo.get("intakeRight");

        foundationServoLeft = hardwareMap.servo.get("foundationMoverLeft");
        foundationServoLeft.setPosition(0.69); // Reset position
        foundationServoRight = hardwareMap.servo.get("foundationMoverRight");
        foundationServoRight.setPosition(0.5);

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, GlobalConfig.TICKS_PER_INCH, GlobalConfig.TICKS_PER_360);

        motorSlideLeft.setTargetPosition(0);
        motorSlideRight.setTargetPosition(0);
//        motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        // MOVEMENT (DRIVING/STRAFING AND ROTATION)
        double strafe_x = -gamepad1.left_stick_x, strafe_y = gamepad1.left_stick_y, rotate_power = -gamepad1.right_stick_x;

        // SLOW MODE STRAFE WITH D-PAD
        if (gamepad1.dpad_up) strafe_y = -SLOW_MODE;
        if (gamepad1.dpad_right) strafe_x = -SLOW_MODE;
        if (gamepad1.dpad_down) strafe_y = SLOW_MODE;
        if (gamepad1.dpad_left) strafe_x = SLOW_MODE;

        // SLOW MODE ROTATION WITH X AND B BUTTONS
        if (gamepad1.b) rotate_power = -SLOW_MODE;
        if (gamepad1.x) rotate_power = SLOW_MODE;

        Coordinate strafe = Coordinate.fromXY(strafe_x, strafe_y);
        mecanumDrive.setStrafe(strafe);
        mecanumDrive.setRotationPower(rotate_power);

        // FOUNDATION MOVER
        if (gamepad1.left_bumper) {
            foundationServoLeft.setPosition(0.52);
            foundationServoRight.setPosition(0.67);
        } else if (gamepad1.right_bumper) {
            foundationServoLeft.setPosition(0.69);
            foundationServoRight.setPosition(0.5);
        }
        // INTAKE AND OUTPUT
        double intakeServoPower = 0;

        if (gamepad2.dpad_left)
            intakeServoPower = -1;
        else if (gamepad2.dpad_right)
            intakeServoPower = 1;

        intakeServoLeft.setPower(intakeServoPower);
        intakeServoRight.setPower(intakeServoPower);

        if (intakeServoPower == 0) {
            // ROTATE STONES
            // Left and right d-pad spins servos on intake both the same way to rotate stones
            if (gamepad2.right_trigger > 0)
                intakeServoPower = gamepad2.right_trigger;
            else if (gamepad2.left_trigger > 0)
                intakeServoPower = -gamepad2.left_trigger;

            intakeServoLeft.setPower(intakeServoPower);
            intakeServoRight.setPower(-intakeServoPower);
        }

        // SLIDES
//        if (gamepad2.a || gamepad2.y) {
//            if (moveLiftMotor == null || moveLiftMotor.isDone()) {
//                asyncExecutor.submit(moveLift);
//            }
//        }

        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double slidePower = -gamepad2.left_stick_y;

        if (Math.abs(slidePower) <= 0.04)
            slidePower = 0.1;
        else if (slidePower < -0.04)

            slidePower = 0.0075;

        telemetry.addData("slidePower", slidePower);
        telemetry.update();

        motorSlideLeft.setPower(slidePower);
        motorSlideRight.setPower(slidePower);

    }

    private void checkForInterrupt() throws InterruptedException {
        if (Thread.interrupted())
            throw new InterruptedException();
    }
}

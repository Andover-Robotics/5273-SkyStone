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
    private Servo sideClawArmLeft, sideClawFingerLeft, sideClawArmRight, sideClawFingerRight, capstoneMover, capstoneHolder;
    private final double STRAFE_SLOW_MODE = 0.4, ROTATE_SLOW_MODE = 0.25, LIFT_SLOW_MODE = 0.3;

    private boolean leftArmOpen, leftFingerOpen, rightArmOpen, rightFingerOpen;

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

            motorSlideLeft.setPower(LIFT_SLOW_MODE);
            motorSlideRight.setPower(LIFT_SLOW_MODE);

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
        foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_UP); // Reset position

        foundationServoRight = hardwareMap.servo.get("foundationMoverRight");
        foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_UP);

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, GlobalConfig.TICKS_PER_INCH, GlobalConfig.TICKS_PER_360);

        motorSlideLeft.setTargetPosition(0);
        motorSlideRight.setTargetPosition(0);
//        motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sideClawArmLeft = hardwareMap.servo.get("sideClawArmLeft");
        sideClawFingerLeft = hardwareMap.servo.get("sideClawFingerLeft");

        sideClawArmRight = hardwareMap.servo.get("sideClawArmRight");
        sideClawFingerRight = hardwareMap.servo.get("sideClawFingerRight");

        sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
        sideClawFingerLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_FINGER_OPEN);

        sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
        sideClawFingerRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_FINGER_OPEN);

        capstoneMover = hardwareMap.servo.get("capstoneMover");

        capstoneMover.setPosition(GlobalConfig.CAPSTONE_MOVER_HOLD);

        capstoneHolder = hardwareMap.servo.get("capstoneHolder");

        capstoneHolder.setPosition(GlobalConfig.CAPSTONE_HOLDER_LOCK);

        leftArmOpen = true;
        leftFingerOpen = true;
        rightArmOpen = true;
        rightFingerOpen = true;
    }

    @Override
    public void loop() {
        // MOVEMENT (DRIVING/STRAFING AND ROTATION)
        double gp1LeftStickX = -gamepad1.left_stick_x, gp1LeftStickY = gamepad1.left_stick_y, rotate_power = -gamepad1.right_stick_x, gp2RightTrigger = gamepad2.right_trigger, gp2LeftTrigger = gamepad2.left_trigger, gp2LeftStickY = gamepad2.left_stick_y, gp2RightStickX = gamepad2.right_stick_x;
        boolean gp1RightBumper = gamepad1.right_bumper, gp1LeftBumper = gamepad1.left_bumper, gp2RightBumper = gamepad2.right_bumper, gp2LeftBumper = gamepad2.left_bumper, gp2DPUp = gamepad2.dpad_up, gp2DPDown = gamepad2.dpad_down, gp1Start = gamepad1.start, gp2X = gamepad2.x, gp2Y = gamepad2.y, gp2B = gamepad2.b, gp2A = gamepad2.a, gp1DPUp = gamepad1.dpad_up, gp1DPRight = gamepad1.dpad_right, gp1DPLeft = gamepad1.dpad_left, gp1DPDown = gamepad1.dpad_down, gp1X = gamepad1.x, gp1B = gamepad1.b, gp2RightJoystick = gamepad2.right_stick_button, gp2LeftJoystick = gamepad2.left_stick_button;

        if (gp1Start) {
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
            sideClawFingerLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_FINGER_OPEN);
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
            sideClawFingerRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_FINGER_OPEN);

            leftArmOpen = true;
            leftFingerOpen = true;
            rightArmOpen = true;
            rightFingerOpen = true;
        }

        if (gp2X) {
            sideClawArmLeft.setPosition(leftArmOpen ? GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN : GlobalConfig.LEFT_SIDE_CLAW_ARM_UP);
            leftArmOpen = !leftArmOpen;
        }

        if (gp2Y) {
            sideClawFingerLeft.setPosition(leftFingerOpen ? GlobalConfig.LEFT_SIDE_CLAW_FINGER_CLOSE : GlobalConfig.LEFT_SIDE_CLAW_FINGER_OPEN);
            leftFingerOpen = !leftFingerOpen;
        }

        if (gp2B) {
            sideClawArmRight.setPosition(rightArmOpen ? GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN : GlobalConfig.RIGHT_SIDE_CLAW_ARM_UP);
            rightArmOpen = !rightArmOpen;
        }

        if (gp2A) {
            sideClawFingerRight.setPosition(rightFingerOpen ? GlobalConfig.RIGHT_SIDE_CLAW_FINGER_CLOSE : GlobalConfig.RIGHT_SIDE_CLAW_FINGER_OPEN);
            rightFingerOpen = !rightFingerOpen;
        }

        if(gp2RightBumper) {
            sideClawArmRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_ARM_DOWN);
            sideClawFingerRight.setPosition(GlobalConfig.RIGHT_SIDE_CLAW_FINGER_OUT);
            rightArmOpen = false;
            rightFingerOpen = false;
        }

        if(gp2LeftBumper) {
            sideClawArmLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_ARM_DOWN);
            sideClawFingerLeft.setPosition(GlobalConfig.LEFT_SIDE_CLAW_FINGER_OUT);
            leftArmOpen = false;
            leftFingerOpen = false;
        }

        // SLOW MODE STRAFE WITH D-PAD
        if (gp1DPUp) gp1LeftStickY = -STRAFE_SLOW_MODE;
        else if (gp1DPDown) gp1LeftStickY = STRAFE_SLOW_MODE;
        if (gp1DPRight) gp1LeftStickX = -STRAFE_SLOW_MODE;
        else if (gp1DPLeft) gp1LeftStickX = STRAFE_SLOW_MODE;

        // SLOW MODE ROTATION WITH X AND B BUTTONS
        if (gp1B) rotate_power = -ROTATE_SLOW_MODE;
        else if (gp1X) rotate_power = ROTATE_SLOW_MODE;

        Coordinate strafe = Coordinate.fromXY(gp1LeftStickX, gp1LeftStickY);
        if (Math.abs(strafe.getPolarDistance()) >= 0.02) {
            mecanumDrive.setStrafeRotation(strafe, strafe.getPolarDistance(), rotate_power);
        } else {
            mecanumDrive.setRotationPower(rotate_power);
        }

        // FOUNDATION MOVER
        if (gp1RightBumper) {
            foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_UP);
            foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_UP);
        } else if (gp1LeftBumper) {
            foundationServoLeft.setPosition(GlobalConfig.FOUNDATION_SERVO_LEFT_DOWN);
            foundationServoRight.setPosition(GlobalConfig.FOUNDATION_SERVO_RIGHT_DOWN);
        }
        // INTAKE AND OUTPUT
        double intakeServoPower = 0;

        if (gp2RightTrigger > 0.05)
            intakeServoPower = 1;
        else if (gp2LeftTrigger > 0.05)
            intakeServoPower = -1;

        intakeServoLeft.setPower(intakeServoPower);
        intakeServoRight.setPower(-intakeServoPower);

        if (intakeServoPower == 0) {
            // ROTATE STONES
            // Left and right d-pad spins servos on intake both the same way to rotate stones
            if (gp2RightStickX == -1)
                intakeServoPower = -1;
            else if (gp2RightStickX == 1)
                intakeServoPower = 1;

            intakeServoLeft.setPower(intakeServoPower);
            intakeServoRight.setPower(intakeServoPower);
        }
        // SLIDES
//        if (gamepad2.a || gamepad2.y) {
//            if (moveLiftMotor == null || moveLiftMotor.isDone()) {
//                asyncExecutor.submit(moveLift);
//            }
//        }

        motorSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double slidePower = -gp2LeftStickY * 0.8;

        if (Math.abs(slidePower) <= 0.04)
            slidePower = 0.15;
        else if (slidePower < -0.04)
            slidePower = 0.0075;

        telemetry.addData("slidePower", slidePower);
        telemetry.update();

        motorSlideLeft.setPower(slidePower);
        motorSlideRight.setPower(slidePower);

        //Capstone Mover
        if(gp2DPUp) capstoneMover.setPosition(GlobalConfig.CAPSTONE_MOVER_DROP);
        if(gp2DPDown) capstoneMover.setPosition(GlobalConfig.CAPSTONE_MOVER_HOLD);
        if(gp2RightJoystick) capstoneHolder.setPosition(GlobalConfig.CAPSTONE_HOLDER_UNLOCK); //unlock
        if (gp2LeftJoystick) capstoneHolder.setPosition(GlobalConfig.CAPSTONE_HOLDER_LOCK); //lock

        /*if(gamepad2.y) sideClawArmPos += 0.01;
        if(gamepad2.b) sideClawArmPos -= 0.01;
        if(gamepad2.x) sideClawFingerPos += 0.01;
        if(gamepad2.a) sideClawFingerPos -= 0.01;

        sideClawArmLeft.setPosition(sideClawArmPos);
        sideClawFingerLeft.setPosition(sideClawFingerPos);*/
    }

    private void checkForInterrupt() throws InterruptedException {
        if (Thread.interrupted())
            throw new InterruptedException();
    }
}

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "DanceBot TeleOp", group = "Demo")
public class DanceBotTeleOp extends LinearOpMode {
    private static final String TAG = "DanceBotTeleOp";
    private static final double NORMAL_SCALE = 1.00;
    private static final double SLOW_SCALE = 0.30;
    private static final double STICK_DEAD_ZONE = 0.08;

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;
    private boolean fourMotorDrive;

    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor configuredBackRight = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor configuredBackLeft = hardwareMap.tryGet(DcMotor.class, "back_left");
        DcMotor configuredFrontLeft = hardwareMap.tryGet(DcMotor.class, "front_left");
        intake = hardwareMap.tryGet(DcMotor.class, "intake");

        if ((configuredBackLeft == null) != (configuredFrontLeft == null)) {
            throw new IllegalStateException(
                    "Drive configuration must contain front_right/back_right or all four named drive motors");
        }

        fourMotorDrive = configuredBackLeft != null;
        if (fourMotorDrive) {
            backRight = configuredBackRight;
            backLeft = configuredBackLeft;
            frontLeft = configuredFrontLeft;
        } else {
            frontLeft = configuredBackRight;
        }

        setDirections();
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (intake != null) {
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setPower(0.0);
        }
        stopMotors();

        log("Initialized %s robot-centric mecanum drive; battery=%.2fV",
                fourMotorDrive ? "4-motor (FR=0 BR=1 BL=2 FL=3)" : "2-motor (R=0 L=1)",
                batteryVoltage());
        telemetry.addData("Status", "Ready (%d motors)", fourMotorDrive ? 4 : 2);
        telemetry.addData("Drive", "LS: drive/strafe; RS X: turn; RB: slow");
        telemetry.addData("Intake", "Either gamepad RT: in; LT: reverse; B: stop");
        telemetry.addData("Battery", "%.2f V", batteryVoltage());
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                double scale = gamepad1.right_bumper ? SLOW_SCALE : NORMAL_SCALE;

                // Recommended robot-centric mecanum map: left stick translates,
                // while right-stick X rotates the robot.
                double drive = applyDeadZone(-gamepad1.left_stick_y);
                double strafe = fourMotorDrive ? applyDeadZone(gamepad1.left_stick_x) : 0.0;
                double turn = applyDeadZone(gamepad1.right_stick_x);

                double intakePower = getIntakePower();

                double frontLeftPower = drive + strafe + turn;
                double frontRightPower = drive - strafe - turn;
                double backLeftPower = drive - strafe + turn;
                double backRightPower = drive + strafe - turn;

                // Normalize all four wheels together so the requested direction is preserved.
                double maxMagnitude = Math.max(1.0,
                        Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
                frontLeftPower = Range.clip((frontLeftPower / maxMagnitude) * scale, -1.0, 1.0);
                frontRightPower = Range.clip((frontRightPower / maxMagnitude) * scale, -1.0, 1.0);
                backLeftPower = Range.clip((backLeftPower / maxMagnitude) * scale, -1.0, 1.0);
                backRightPower = Range.clip((backRightPower / maxMagnitude) * scale, -1.0, 1.0);

                setDrivePowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
                if (intake != null) {
                    intake.setPower(intakePower);
                }

                telemetry.addData("Mode", gamepad1.right_bumper ? "Slow" : "Normal");
                telemetry.addData("Speed", "%.2f", drive);
                telemetry.addData("Turn", "%.2f", turn);
                telemetry.addData("Strafe", "%.2f", strafe);
                telemetry.addData("Front Left", "%.2f", frontLeftPower);
                telemetry.addData("Front Right", "%.2f", frontRightPower);
                telemetry.addData("Back Left", "%.2f", backLeftPower);
                telemetry.addData("Back Right", "%.2f", backRightPower);
                telemetry.addData("Intake", intake == null ? "Not configured" : String.format("%.2f", intakePower));
                telemetry.addData("Battery", "%.2f V", batteryVoltage());
                telemetry.update();

                idle();
            }
        } finally {
            stopMotors();
            if (intake != null) {
                intake.setPower(0.0);
            }
            log("Stopped motors");
        }
    }

    private void setDirections() {
        if (fourMotorDrive) {
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            // Two-wheel DanceBot (config: dancebot-2wd.xml). Both motors sit on one
            // crossbar with shafts pointing outward, so they are mirror images.
            // Measured with DanceBot Motor Test (raw positive power, no direction set):
            //   port 0 (front_right, right side): positive rolls BACKWARD -> REVERSE
            //   port 1 (back_right,  left side):  positive rolls FORWARD  -> FORWARD
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    private void setRunMode(DcMotor.RunMode mode) {
        frontRight.setMode(mode);
        frontLeft.setMode(mode);
        if (fourMotorDrive) {
            backLeft.setMode(mode);
            backRight.setMode(mode);
        }
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontRight.setZeroPowerBehavior(behavior);
        frontLeft.setZeroPowerBehavior(behavior);
        if (fourMotorDrive) {
            backLeft.setZeroPowerBehavior(behavior);
            backRight.setZeroPowerBehavior(behavior);
        }
    }

    private double applyDeadZone(double input) {
        double magnitude = Math.abs(input);
        if (magnitude <= STICK_DEAD_ZONE) {
            return 0.0;
        }
        return Math.copySign((magnitude - STICK_DEAD_ZONE) / (1.0 - STICK_DEAD_ZONE), input);
    }

    private double getIntakePower() {
        if (intake == null || gamepad1.b || gamepad2.b) {
            return 0.0;
        }

        double gamepad2Power = applyDeadZone(gamepad2.right_trigger)
                - applyDeadZone(gamepad2.left_trigger);
        if (gamepad2Power != 0.0) {
            return gamepad2Power;
        }

        return applyDeadZone(gamepad1.right_trigger)
                - applyDeadZone(gamepad1.left_trigger);
    }

    private void setDrivePowers(double frontLeftPower, double backLeftPower,
                                double frontRightPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        if (fourMotorDrive) {
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        }
    }

    private void stopMotors() {
        setDrivePowers(0.0, 0.0, 0.0, 0.0);
    }

    private double batteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0.0) {
                result = Math.min(result, voltage);
            }
        }
        return result == Double.POSITIVE_INFINITY ? 0.0 : result;
    }

    private void log(String format, Object... args) {
        String message = String.format(format, args);
        Log.i(TAG, message);
        RobotLog.ii(TAG, message);
    }
}

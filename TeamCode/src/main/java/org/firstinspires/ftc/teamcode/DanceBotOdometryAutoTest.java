package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Low-speed closed-loop drive test using the calibrated three-wheel odometry. */
@Autonomous(name = "DanceBot Odometry Auto Test", group = "Diagnostics")
public class DanceBotOdometryAutoTest extends LinearOpMode {
    private static final double PARALLEL_TICKS_PER_INCH = 1757.55;
    private static final double CENTER_TICKS_PER_INCH = 1768.34;
    // Effective spacing calibrated against the correctly oriented Control Hub IMU.
    private static final double LEFT_POD_Y_INCHES = 4.14345;
    private static final double RIGHT_POD_Y_INCHES = -4.14345;
    private static final double CENTER_POD_X_INCHES = 0.0107;

    private static final double TRANSLATION_KP = 0.055;
    private static final double HEADING_KP = 0.75;
    private static final double MAX_DRIVE_POWER = 0.25;
    private static final double MAX_TURN_POWER = 0.20;
    private static final double MIN_DRIVE_POWER = 0.10;
    // Enough authority for small heading corrections to overcome drivetrain friction.
    private static final double MIN_TURN_POWER = 0.11;
    // Low-speed static friction leaves up to roughly 1.8 inches of residual error.
    private static final double POSITION_TOLERANCE_INCHES = 2.0;
    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final double WAYPOINT_TIMEOUT_SECONDS = 12.0;
    private static final double SETTLE_SECONDS = 0.40;
    private static final double MAX_ERROR_GROWTH_INCHES = 4.0;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private int previousLeft;
    private int previousRight;
    private int previousCenter;
    private double x;
    private double y;
    private double heading;
    private double commandedFrontLeft;
    private double commandedFrontRight;
    private double commandedBackLeft;
    private double commandedBackRight;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setBrake();
        stopDrive();
        resetPose();

        telemetry.addLine("LOW-SPEED TEST: robot will move automatically");
        telemetry.addLine("Clear a 5 x 5 ft area and stay ready to press STOP");
        telemetry.addLine("Path: 24-in square, then restore starting heading");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        resetPose();

        try {
            // Trace a 24-inch square. Translation targets are field-relative.
            if (!driveToPose(24, 0, 0, "Forward 24")) return;
            if (!driveToPose(24, 24, 0, "Left 24")) return;
            if (!driveToPose(24, 24, 90, "Turn CCW 90")) return;
            if (!driveToPose(0, 24, 90, "Across 24")) return;
            if (!driveToPose(0, 24, 180, "Turn CCW 180")) return;
            if (!driveToPose(0, 0, 180, "Return 24")) return;
            driveToPose(0, 0, 0, "Restore heading");
        } finally {
            stopDrive();
        }

        while (opModeIsActive()) {
            updatePose();
            showTelemetry("Complete", 0, 0, 0, 0);
            telemetry.addLine("Press STOP when readings are recorded");
            telemetry.update();
            idle();
        }
    }

    private boolean driveToPose(double targetX, double targetY,
                                double targetHeadingDegrees, String label) {
        double targetHeading = Math.toRadians(targetHeadingDegrees);
        ElapsedTime timeout = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        boolean insideTolerance = false;
        double initialPositionError = Math.hypot(targetX - x, targetY - y);

        while (opModeIsActive() && timeout.seconds() < WAYPOINT_TIMEOUT_SECONDS) {
            updatePose();
            double errorX = targetX - x;
            double errorY = targetY - y;
            double errorHeading = normalizeRadians(targetHeading - heading);
            double positionError = Math.hypot(errorX, errorY);

            // Abort a wrong-direction or failed-feedback move well before the timeout.
            if (timeout.seconds() > 0.75
                    && positionError > initialPositionError + MAX_ERROR_GROWTH_INCHES) {
                stopDrive();
                showTelemetry("ABORT: moving away from target", targetX, targetY,
                        targetHeadingDegrees, timeout.seconds());
                telemetry.update();
                sleep(1500);
                return false;
            }

            if (positionError <= POSITION_TOLERANCE_INCHES
                    && Math.abs(errorHeading) <= HEADING_TOLERANCE_RADIANS) {
                stopDrive();
                if (!insideTolerance) {
                    settled.reset();
                    insideTolerance = true;
                }
                if (settled.seconds() >= SETTLE_SECONDS) return true;
            } else {
                insideTolerance = false;

                // Rotate the field error into the robot's forward/left frame.
                double cos = Math.cos(heading);
                double sin = Math.sin(heading);
                double robotForwardError = errorX * cos + errorY * sin;
                double robotLeftError = -errorX * sin + errorY * cos;
                double forward = Range.clip(robotForwardError * TRANSLATION_KP,
                        -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
                // This drivetrain's positive mecanum strafe mix moves physically right.
                double left = Range.clip(-robotLeftError * TRANSLATION_KP,
                        -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

                // Maintain enough translation power to overcome static friction.
                if (positionError <= POSITION_TOLERANCE_INCHES) {
                    forward = 0.0;
                    left = 0.0;
                } else {
                    double translationPower = Math.hypot(forward, left);
                    if (translationPower > 0.0 && translationPower < MIN_DRIVE_POWER) {
                        double boost = MIN_DRIVE_POWER / translationPower;
                        forward *= boost;
                        left *= boost;
                    }
                }

                // Positive wheel-mix turn is clockwise; odometry heading is CCW-positive.
                double turn = Range.clip(-errorHeading * HEADING_KP,
                        -MAX_TURN_POWER, MAX_TURN_POWER);
                // Maintain enough turn power to overcome static friction.
                if (Math.abs(errorHeading) <= HEADING_TOLERANCE_RADIANS) {
                    turn = 0.0;
                } else if (Math.abs(turn) < MIN_TURN_POWER) {
                    turn = Math.copySign(MIN_TURN_POWER, turn);
                }
                setMecanum(forward, left, turn);
            }

            showTelemetry(label, targetX, targetY, targetHeadingDegrees, timeout.seconds());
            telemetry.update();
            idle();
        }

        stopDrive();
        return false;
    }

    private void updatePose() {
        int currentLeft = readLeft();
        int currentRight = readRight();
        int currentCenter = readCenter();
        double deltaLeft = (currentLeft - previousLeft) / PARALLEL_TICKS_PER_INCH;
        double deltaRight = (currentRight - previousRight) / PARALLEL_TICKS_PER_INCH;
        double deltaCenter = (currentCenter - previousCenter) / CENTER_TICKS_PER_INCH;
        previousLeft = currentLeft;
        previousRight = currentRight;
        previousCenter = currentCenter;

        double deltaHeading = (deltaRight - deltaLeft)
                / (LEFT_POD_Y_INCHES - RIGHT_POD_Y_INCHES);
        double forward = (deltaLeft + deltaRight) / 2.0;
        double lateral = deltaCenter - CENTER_POD_X_INCHES * deltaHeading;
        double midpoint = heading + deltaHeading / 2.0;
        x += forward * Math.cos(midpoint) - lateral * Math.sin(midpoint);
        y += forward * Math.sin(midpoint) + lateral * Math.cos(midpoint);
        heading = normalizeRadians(heading + deltaHeading);
    }

    private void resetPose() {
        previousLeft = readLeft();
        previousRight = readRight();
        previousCenter = readCenter();
        x = y = heading = 0.0;
    }

    // Motor direction reverses the left encoder sign, so undo it for calibrated raw pod signs.
    private int readLeft() { return -backLeft.getCurrentPosition(); }
    private int readRight() { return -frontRight.getCurrentPosition(); }
    // The back-right motor direction reverses its external encoder reading, so undo it
    // to preserve the center-pod sign used during calibration (physical left is positive).
    private int readCenter() { return -backRight.getCurrentPosition(); }

    private void setMecanum(double forward, double left, double turn) {
        double fl = forward + left + turn;
        double fr = forward - left - turn;
        double bl = forward - left + turn;
        double br = forward + left - turn;
        double scale = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        commandedFrontLeft = fl / scale;
        commandedFrontRight = fr / scale;
        commandedBackLeft = bl / scale;
        commandedBackRight = br / scale;
        frontLeft.setPower(commandedFrontLeft);
        frontRight.setPower(commandedFrontRight);
        backLeft.setPower(commandedBackLeft);
        backRight.setPower(commandedBackRight);
    }

    private void showTelemetry(String step, double targetX, double targetY,
                               double targetHeading, double seconds) {
        telemetry.addData("Step", step);
        telemetry.addData("Target X / Y", "%.1f / %.1f in", targetX, targetY);
        telemetry.addData("Target heading", "%.1f deg", targetHeading);
        telemetry.addData("Pose X / Y", "%+.2f / %+.2f in", x, y);
        telemetry.addData("Heading", "%+.1f deg", Math.toDegrees(heading));
        telemetry.addData("Power FL / FR", "%+.2f / %+.2f",
                commandedFrontLeft, commandedFrontRight);
        telemetry.addData("Power BL / BR", "%+.2f / %+.2f",
                commandedBackLeft, commandedBackRight);
        telemetry.addData("Step time", "%.1f / %.1f sec", seconds, WAYPOINT_TIMEOUT_SECONDS);
    }

    private void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode); frontRight.setMode(mode);
        backLeft.setMode(mode); backRight.setMode(mode);
    }

    private void setBrake() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void stopDrive() { setMecanum(0, 0, 0); }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}

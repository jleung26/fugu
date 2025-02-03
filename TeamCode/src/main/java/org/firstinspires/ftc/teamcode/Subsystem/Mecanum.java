package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.PinpointManager;

@Config
public class Mecanum {
    OpMode opmode;
    private DcMotorEx Fl, Fr, Bl, Br;
    private volatile double prevFrontLeftPower, prevBackLeftPower, prevFrontRightPower, prevBackRightPower;
    public PinpointManager pinpoint = new PinpointManager();

    public static double SLOW_MODE_FACTOR = 0.5;
    public static double CACHING_THRESHOLD = 0.005;
    public static double SCALING_EXPONENT = 2.2;

    private volatile boolean angleLockBool = false, slowModeBool = false;

    public static double Kp = 0.008;
    public static double Kd = 0;

    // variables for later modification
    public double targetAngle = 0;
    // PID
    private double error, lastError;
    ElapsedTime timer = new ElapsedTime();

    public Mecanum() {}

    public void initialize(OpMode opmode) {
        this.opmode = opmode;
        this.Fl = opmode.hardwareMap.get(DcMotorEx.class, "leftFront");
        this.Fr = opmode.hardwareMap.get(DcMotorEx.class, "rightFront");
        this.Bl = opmode.hardwareMap.get(DcMotorEx.class, "leftRear");
        this.Br = opmode.hardwareMap.get(DcMotorEx.class, "rightRear");

        Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        Fl.setDirection(DcMotorSimple.Direction.REVERSE);
        Bl.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint.initialize(opmode);
    }

    public void operateTeleOp() {
        pinpoint.operateTeleOp();

        if (angleLockBool) {
            driveRobotCentric(scaleJoystick(opmode.gamepad1.left_stick_x), scaleJoystick(-opmode.gamepad1.left_stick_y), -PDTurning(targetAngle, pinpoint.relativeNormalizedHeading), slowModeBool);
        }
        else {
            driveRobotCentric(scaleJoystick(opmode.gamepad1.left_stick_x), scaleJoystick(-opmode.gamepad1.left_stick_y), opmode.gamepad1.right_stick_x, slowModeBool);
        }
    }

    public void operateTesting() {
        pinpoint.operateTeleOp();

        // for testing PD auto orienting
        // auto rotate to angle with PID test
        if (opmode.gamepad1.right_trigger > 0.1) {
            driveRobotCentric(scaleJoystick(opmode.gamepad1.left_stick_x), scaleJoystick(-opmode.gamepad1.left_stick_y), -PDTurning(targetAngle, pinpoint.relativeNormalizedHeading), false);
        }
        else {
            slowModeBool = opmode.gamepad1.left_trigger > 0.1;
            driveRobotCentric(scaleJoystick(opmode.gamepad1.left_stick_x), scaleJoystick(-opmode.gamepad1.left_stick_y), opmode.gamepad1.right_stick_x, slowModeBool);
        }

        opmode.telemetry.addData("Current Scaling Exponent: input^", SCALING_EXPONENT);
        opmode.telemetry.addData("current heading: ", pinpoint.relativeNormalizedHeading);
        opmode.telemetry.addData("absolute heading: ", pinpoint.normalizedHeading);
        opmode.telemetry.addData("target heading: ", targetAngle);
        opmode.telemetry.addData("gyro offset: ", pinpoint.offset);
        opmode.telemetry.addData("PD calculated rx [-1,1]: ", PDTurning(targetAngle, pinpoint.relativeNormalizedHeading));
        opmode.telemetry.addData("normalized error: ", normalizeError(targetAngle - pinpoint.relativeNormalizedHeading));
    }

    public void driveRobotCentric(double x, double y, double rx, boolean slowmode) {
        x = x * (slowmode ? SLOW_MODE_FACTOR: 1);
        y = y * (slowmode ? SLOW_MODE_FACTOR: 1);
        rx = rx * (slowmode ? SLOW_MODE_FACTOR * 0.75 : 1);
        // calculating output
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // very un-wrappered power caching
        if (comparePower(prevFrontLeftPower, frontLeftPower)) { Fl.setPower(frontLeftPower); }
        if (comparePower(prevBackLeftPower, backLeftPower)) { Bl.setPower(backLeftPower); }
        if (comparePower(prevFrontRightPower, frontRightPower)) { Fr.setPower(frontRightPower); }
        if (comparePower(prevBackRightPower, backRightPower)) { Br.setPower(backRightPower); }

        // assigns for next loop
        prevFrontLeftPower = frontLeftPower;
        prevBackLeftPower = backLeftPower;
        prevFrontRightPower = frontRightPower;
        prevBackRightPower = backRightPower;
    }

    public void driveFieldCentric(double x, double y, double rx, double heading, boolean slowmode) {
        x = x * (slowmode ? SLOW_MODE_FACTOR: 1);
        y = y * (slowmode ? SLOW_MODE_FACTOR: 1);
        rx = rx * (slowmode ? SLOW_MODE_FACTOR * 0.75 : 1);
        // calculating output
        double headingRads = -Math.toRadians(heading);
        double rotX = y * Math.cos(headingRads) + x * Math.sin(headingRads);
        double rotY = y * Math.sin(headingRads) - x * Math.cos(headingRads);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // very un-wrappered power caching
        if (comparePower(prevFrontLeftPower, frontLeftPower)) { Fl.setPower(frontLeftPower); }
        if (comparePower(prevBackLeftPower, backLeftPower)) { Bl.setPower(backLeftPower); }
        if (comparePower(prevFrontRightPower, frontRightPower)) { Fr.setPower(frontRightPower); }
        if (comparePower(prevBackRightPower, backRightPower)) { Br.setPower(backRightPower); }

        // assigns for next loop
        prevFrontLeftPower = frontLeftPower;
        prevBackLeftPower = backLeftPower;
        prevFrontRightPower = frontRightPower;
        prevBackRightPower = backRightPower;
    }

    // checks if powers are different enough
    public boolean comparePower(double prevPower, double currentPower) {
        return Math.abs(currentPower - prevPower) >= CACHING_THRESHOLD;
    }

    public double PDTurning(double targetHeading, double currentHeading) {
        // calculate the error
        error = normalizeError(targetHeading - currentHeading);

        double derivative = (error - lastError) / timer.seconds();

        double output = Math.max(-1, Math.min(1, (Kp * error) + (Kd * derivative)));
        // square root PID, if robot is too fat and has too much inertia to fix small error
        // Jayden will diddle around with PID auto turning on his own time
        // FLOAT mode might make tuning very hard :noooo:, since it could depend on robot's strafe movement, and a lot on mass
        // double output = Math.signum(output) * Math.min(1, Math.sqrt(Math.abs(output)));
        // double output = Math.signum(output) * Math.min(1, Math.pow(Math.abs(output), 0.7)); // 0.3-0.7 are all options too depends on robot fatness

        // reset stuff for next time
        timer.reset();
        lastError = error;

        return output;
    }

    private double normalizeError(double error) {
        // Normalize error to -180 to 180
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    public double scaleJoystick(double input) {
        return Math.signum(input) * Math.pow(Math.abs(input), SCALING_EXPONENT);
    }

    public void setAngleLockTrue() { angleLockBool = true; }
    public void setAngleLockFalse() { angleLockBool = true; }
    public void setSlowModeTrue() { slowModeBool = true; }
    public void setSlowModeFalse() { slowModeBool = false; }
}
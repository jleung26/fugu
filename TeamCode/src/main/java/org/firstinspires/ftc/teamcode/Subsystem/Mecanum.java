package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.PinpointManager;

@Config
public class PinpointMecanum {
    OpMode opmode;
    private DcMotorEx Fl, Fr, Bl, Br;
    private volatile double prevFrontLeftPower, prevBackLeftPower, prevFrontRightPower, prevBackRightPower;
    private PinpointManager pinpoint = new PinpointManager();

    public static double SLOW_MODE_FACTOR = 0.5;
    public static double CACHING_THRESHOLD = 0.005;
    public static double SCALING_EXPONENT = 1;

    private volatile boolean angleLockBool = false, slowModeBool = false;

    public static double Kp = 0.01;
    public static double Kd = 0;

    // variables for later modification
    private double targetAngle = 0;
    // PID
    private double error, lastError;
    ElapsedTime timer = new ElapsedTime();

    public PinpointMecanum() {}

    public void initialize(OpMode opmode) {
        this.opmode = opmode;
        this.Fl = opmode.hardwareMap.get(DcMotorEx.class, "");
        this.Fr = opmode.hardwareMap.get(DcMotorEx.class, "");
        this.Bl = opmode.hardwareMap.get(DcMotorEx.class, "");
        this.Br = opmode.hardwareMap.get(DcMotorEx.class, "");

        Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Fl.setDirection(DcMotorSimple.Direction.REVERSE);
        Bl.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint.initialize(opmode);
    }

    public void operateTeleOp() {
        pinpoint.operateTeleOp();

        if (angleLockBool) {
            if (slowModeBool) { driveRobotCentric(Math.pow(opmode.gamepad1.left_stick_x, SCALING_EXPONENT) * SLOW_MODE_FACTOR, -Math.pow(opmode.gamepad1.left_stick_y, SCALING_EXPONENT) * SLOW_MODE_FACTOR, PDTurning(pinpoint.relativeNormalizedHeading, targetAngle)  * SLOW_MODE_FACTOR); }
            else { driveRobotCentric(Math.pow(opmode.gamepad1.left_stick_x, SCALING_EXPONENT), -Math.pow(opmode.gamepad1.left_stick_y, SCALING_EXPONENT), PDTurning(pinpoint.relativeNormalizedHeading, targetAngle)); }
        }
        else {
            if (slowModeBool) { driveRobotCentric(Math.pow(opmode.gamepad1.left_stick_x, SCALING_EXPONENT) * SLOW_MODE_FACTOR, -Math.pow(opmode.gamepad1.left_stick_y, SCALING_EXPONENT) * SLOW_MODE_FACTOR, Math.pow(opmode.gamepad1.right_stick_x, SCALING_EXPONENT) * SLOW_MODE_FACTOR); }
            else { driveRobotCentric(Math.pow(opmode.gamepad1.left_stick_x, SCALING_EXPONENT), -Math.pow(opmode.gamepad1.left_stick_y, SCALING_EXPONENT), Math.pow(opmode.gamepad1.right_stick_x, SCALING_EXPONENT)); }
        }
    }

    public void operateTesting(OpMode opmode) {
        pinpoint.operateTeleOp();

        // for testing PD auto orienting
        // auto rotate to angle with PID test
        if (opmode.gamepad1.right_trigger > 0.1) {
            driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, PDTurning(pinpoint.relativeNormalizedHeading, targetAngle));
        }
        // slow mode
        else if (opmode.gamepad1.left_trigger > 0.1) {
            driveRobotCentric(opmode.gamepad1.left_stick_x * SLOW_MODE_FACTOR, -opmode.gamepad1.left_stick_y * SLOW_MODE_FACTOR, opmode.gamepad1.right_stick_x * SLOW_MODE_FACTOR);
        }
        // normal drive
        else {
//            driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, opmode.gamepad1.right_stick_x);
            driveRobotCentric(Math.pow(opmode.gamepad1.left_stick_x, SCALING_EXPONENT), -Math.pow(opmode.gamepad1.left_stick_y, SCALING_EXPONENT), Math.pow(opmode.gamepad1.right_stick_x, SCALING_EXPONENT));
        }

        // gyro reset
        if (opmode.gamepad1.b) {pinpoint.softResetYaw();}

        // sets target
        if (opmode.gamepad1.a) {targetAngle = pinpoint.getRelativeNormalizedHeading();}

        opmode.telemetry.addData("Current Scaling Exponent: input^", SCALING_EXPONENT);
        opmode.telemetry.addData("current heading: ", pinpoint.getRelativeNormalizedHeading());
        opmode.telemetry.addData("absolute heading: ", pinpoint.getNormalizedHeading());
        opmode.telemetry.addData("target heading: ", targetAngle);
        opmode.telemetry.addData("gyro offset: ", pinpoint.getOffset());
        opmode.telemetry.addData("PD calculated rx [-1,1]: ", PDTurning(pinpoint.getRelativeNormalizedHeading(), targetAngle));
        opmode.telemetry.addData("normalized error: ", normalizeError(targetAngle - pinpoint.getRelativeNormalizedHeading()));
    }

    public void driveRobotCentric(double x, double y, double rx) {
        // calculating output
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // very un-wrappered power caching
        if (comparePower(prevFrontLeftPower, frontLeftPower)) {
            Fl.setPower(frontLeftPower);
        }
        if (comparePower(prevBackLeftPower, backLeftPower)) {
            Bl.setPower(backLeftPower);
        }
        if (comparePower(prevFrontRightPower, frontRightPower)) {
            Fr.setPower(frontRightPower);
        }
        if (comparePower(prevBackRightPower, backRightPower)) {
            Br.setPower(backRightPower);
        }

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
        // square root PID, if robot is too heavy to fix small error
        // double output = Math.signum(output) * Math.min(1, Math.sqrt(Math.abs(output)));

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

    public void setAngleLockTrue() { angleLockBool = true; }
    public void setAngleLockFalse() { angleLockBool = true; }
    public void setSlowModeTrue() { slowModeBool = true; }
    public void setSlowModeFalse() { slowModeBool = false; }
}
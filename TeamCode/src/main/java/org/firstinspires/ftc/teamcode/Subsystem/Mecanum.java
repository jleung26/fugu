package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.PinpointManager;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class Mecanum {
    OpMode opmode;
    private DcMotorEx Fl, Fr, Bl, Br;
    private volatile double prevFrontLeftPower, prevBackLeftPower, prevFrontRightPower, prevBackRightPower;
    public PinpointManager pinpoint = new PinpointManager();

    public static double SLOW_MODE_FACTOR = 0.5;
    public static double CACHING_THRESHOLD = 0.005;
    public static double SCALING_EXPONENT = 2.2;

    public volatile boolean angleLockBool = false, slowModeBool = false;

    public static double Kp = 0.0007;
    public static double Kd = 0.0001;

    // variables for later modification
    public double targetAngle = 0;
    // PID
    private double lastError;
    ElapsedTime timer = new ElapsedTime();

    public Mecanum() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        this.Fl = robotHardware.Fl;
        this.Fr = robotHardware.Fr;
        this.Bl = robotHardware.Bl;
        this.Br = robotHardware.Br;

        pinpoint.initialize(opmode, robotHardware);
    }

    public void operateTeleOp() {
        pinpoint.operateSimple();

        if (angleLockBool) {
            driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, PDTurning(targetAngle, pinpoint.relativeNormalizedHeading), slowModeBool);
        } else {
            driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, opmode.gamepad1.right_stick_x, slowModeBool);
        }
    }

    public void operateTesting() {
        pinpoint.operateSimple();

        slowModeBool = opmode.gamepad1.left_trigger > 0.1;
        driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, opmode.gamepad1.right_stick_x, slowModeBool);

        // gyro reset
        if (opmode.gamepad1.b) {pinpoint.softResetYaw();}

        // sets target
        if (opmode.gamepad1.a) {targetAngle = pinpoint.relativeNormalizedHeading;}

        setZeroPowerBrake(opmode.gamepad1.left_trigger > 0.1);

        opmode.telemetry.addData("Current Scaling Exponent: input^", SCALING_EXPONENT);
        opmode.telemetry.addData("rel normalized heading: ", pinpoint.relativeNormalizedHeading);
        opmode.telemetry.addData("normalized heading: ", pinpoint.normalizedHeading);
        opmode.telemetry.addData("target heading: ", targetAngle);
        opmode.telemetry.addData("gyro offset: ", pinpoint.offset);
        opmode.telemetry.addData("PD calculated rx [-1,1]: ", PDTurning(targetAngle, pinpoint.relativeNormalizedHeading));
        opmode.telemetry.addData("normalized error: ", normalizeError(targetAngle - pinpoint.relativeNormalizedHeading));
    }

    public void operateTuningPD(TelemetryPacket packet) {
        // changing Kp and Kd values should already update globally

        // just get heading
        pinpoint.operateSimple();


        driveRobotCentric(scaleJoystick(opmode.gamepad1.left_stick_x), scaleJoystick(-opmode.gamepad1.left_stick_y), PDTurning(targetAngle, pinpoint.relativeNormalizedHeading), false);

        if (opmode.gamepad1.a) {
            setTargetToCurrentHeading();
        }

        // not very necessary since running robot centric, but might as well include it
        opmode.telemetry.addData("rel normalized heading: ", pinpoint.relativeNormalizedHeading);
        opmode.telemetry.addData("gyro offset: ", pinpoint.offset);

        // for graphing on dashboard
        packet.put("normalized heading: ", pinpoint.normalizedHeading);
        packet.put("target heading: ", targetAngle);
        packet.put("PD calculated rx [-1,1]: ", getPDPower());
        packet.put("normalized error:", normalizeError(targetAngle - pinpoint.relativeNormalizedHeading));
    }

    public void operateSimple() {
        driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, opmode.gamepad1.right_stick_x, slowModeBool);
    }

    public void operateHang() {
        if (Math.abs(opmode.gamepad2.left_stick_y) > 0.1) {
            Fl.setPower(-opmode.gamepad2.left_stick_y);
            Fr.setPower(-opmode.gamepad2.left_stick_y);
            Bl.setPower(-opmode.gamepad2.left_stick_y);
            Br.setPower(-opmode.gamepad2.left_stick_y);
        } else {
            driveRobotCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, opmode.gamepad1.right_stick_x, false);
        }
    }

    public void driveRobotCentric(double x, double y, double rx, boolean slowmode) {
        x = x * (slowmode ? SLOW_MODE_FACTOR: 1);
        y = y * (slowmode ? SLOW_MODE_FACTOR: 1);
        rx = rx * (slowmode ? SLOW_MODE_FACTOR * 0.6 : 1);
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
        rx = rx * (slowmode ? SLOW_MODE_FACTOR * 0.5 : 1);
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
        double error = normalizeError(currentHeading - targetHeading);

        double derivative = (error - lastError) / timer.seconds();

        double output = (Kp * error) + (Kd * derivative);
        output = Math.signum(output) * Math.pow(Math.abs(output), 0.65);
        output = Math.max(-1, Math.min(1, output));
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

    public void setZeroPowerBrake(boolean brake) {
        if (brake) {
            Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } else {
            Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }
     }

    public void setTargetToCurrentHeading() {
        this.targetAngle = pinpoint.relativeNormalizedHeading;
    }

    public double getPDPower() {
        return PDTurning(targetAngle, pinpoint.relativeNormalizedHeading);
    }

//    public void setAngleLockTrue() { angleLockBool = true; }
//    public void setAngleLockFalse() { angleLockBool = true; }
//    public void setSlowModeTrue() { slowModeBool = true; }
//    public void setSlowModeFalse() { slowModeBool = false; }
}
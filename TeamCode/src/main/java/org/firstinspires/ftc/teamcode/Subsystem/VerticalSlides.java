package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class VerticalSlides {
    OpMode opmode;
    private PIDController controller;
    private DcMotorEx leftSlideMotor, rightSlideMotor;

    // constants
    public static double Kp = 0.01;
    public static double Ki = 0;
    public static double Kd = 0.0003;
    public static double Kg = 0.1;
    public static double CACHING_THRESHOLD = 0.005;
    public static double RETRACTED_THRESHOLD = 10;
    public static int UPPER_LIMIT = 1600; // this is for 1150s
    public static int LOWER_LIMIT = -2;

    // encoder positions
    public static int highBucketPos = 1300;
    public static int lowBucketPos = 500;
    public static int retractedPos = 0;
    public static int scoreClipPos = 555;
//    public static int slamClipPos = 220;

    // declaring variables for later modification
    private volatile double target = 0;
    private volatile double slidePower;
    private volatile double output = 0;
    private volatile double previousOutput = 0;
    public volatile boolean slidesRetracted = true;
    public volatile boolean usePID = true;

    public VerticalSlides() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware, boolean resetEncoders) {
        this.opmode = opmode;

        leftSlideMotor = robotHardware.leftVertMotor;
        rightSlideMotor = robotHardware.rightVertMotor;

        controller = new PIDController(Kp, Ki, Kd);

        if (resetEncoders) {
            leftSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void operate() {
        int currentPos = leftSlideMotor.getCurrentPosition();
        slidesRetracted = currentPos < RETRACTED_THRESHOLD;
        output = controller.calculate(currentPos, target) + (slidesRetracted && target < RETRACTED_THRESHOLD ? 0: Kg);
        output = Math.max(-1, Math.min(1, output));

        // includes power caching
        if (isDifferent(output, previousOutput)) {
            leftSlideMotor.setPower(output);
            rightSlideMotor.setPower(output);
        }
        previousOutput = output;
    }

    public void operateTuning() {
        if (opmode.gamepad2.y) {
            target = highBucketPos;
        } else if (opmode.gamepad2.a) {
            target = retractedPos;
        } else if (opmode.gamepad1.b) {
            target = scoreClipPos;
        }

        int currentPos = leftSlideMotor.getCurrentPosition();

        slidePower = -opmode.gamepad1.right_stick_y * 0.5;
        usePID = opmode.gamepad1.left_trigger > 0.1;
        if (usePID) {
            // use PID
            output = controller.calculate(currentPos, target) + Kg;
            leftSlideMotor.setPower(output);
            rightSlideMotor.setPower(output);
        } else {
            // move manually
            leftSlideMotor.setPower(slidePower);
            rightSlideMotor.setPower(slidePower);
        }

        // if out of range, sets target to back in range
        if (currentPos > UPPER_LIMIT) {
            target = UPPER_LIMIT;
        } else if (currentPos < LOWER_LIMIT) {
            target = LOWER_LIMIT;
        }

        // updates boolean
//        slidesRetracted = currentPos < RETRACTED_THRESHOLD;
        opmode.telemetry.addData("current pos: " , currentPos);
        opmode.telemetry.addData("target: ", target);
        opmode.telemetry.addData("use PID: ", usePID);
    }

    public void operateFix() {
        // manual control
        slidePower = -opmode.gamepad2.left_stick_y;
        leftSlideMotor.setPower(slidePower);
        rightSlideMotor.setPower(slidePower);

        if (opmode.gamepad2.left_stick_button) {
            leftSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void moveToPosition(int targetPos) {target = targetPos;}
    public void raiseToHighBucket() { moveToPosition(highBucketPos);}
    public void raiseToLowBucket()  { moveToPosition(lowBucketPos);}
    public void raiseToScoreClip()   { moveToPosition(scoreClipPos);}
    public void retract()           { moveToPosition(retractedPos);}
//    public void slamToScoreClip()   { moveToPosition(slamClipPos);}

//    public double telemetryMotorPos() {
//        return leftSlideMotor.getCurrentPosition();
//    }
//    public double telemetryTarget() {
//        return target;
//    }
//    public double telemetryOutput() {
//        return output;
//    }

    private boolean isDifferent(double val1, double val2) {
        return Math.abs(val1 - val2) >= CACHING_THRESHOLD;
    }
}


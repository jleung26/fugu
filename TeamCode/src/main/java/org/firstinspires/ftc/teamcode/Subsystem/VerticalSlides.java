package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class VerticalSlides {
    OpMode opmode;
    private PIDController controller;
    private DcMotorEx leftSlideMotor, rightSlideMotor;

    // constants
    public static double Kp = 0.0075;
    public static double Ki = 0;
    public static double Kd = 0.0001;
    public static double Kg = 0.33;
    public static double CACHING_THRESHOLD = 0.005;
    public static double RETRACTED_THRESHOLD = 35;
    public static int UPPER_LIMIT = 1080; // this is for 1150s
    public static int LOWER_LIMIT = -2;

    // encoder positions
    public static int highBucketPos = 950;
    public static int lowBucketPos = 320;
    public static int retractedPos = 0;
    public static int prepClipPos = 575;
    public static int pickupClipPos = 0;

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

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void operate() {
        int currentPos = leftSlideMotor.getCurrentPosition();
        slidesRetracted = currentPos < RETRACTED_THRESHOLD;
        output = controller.calculate(currentPos, target);
        output = (target < currentPos ? -1 : 1) * Math.sqrt(Math.abs(output)) + (slidesRetracted && target < RETRACTED_THRESHOLD ? 0: Kg);
        output = Math.max(-1, Math.min(1, output));

        // includes power caching
        if (isDifferent(output, previousOutput)) {
            leftSlideMotor.setPower(output);
            rightSlideMotor.setPower(output);
        }
        previousOutput = output;
    }

    public void operateTuning(TelemetryPacket packet) {
        controller.setPID(Kp, Ki, Kd);

        if (opmode.gamepad1.y) {
            target = highBucketPos;
        } else if (opmode.gamepad1.a) {
            target = retractedPos;
        } else if (opmode.gamepad1.b) {
            target = prepClipPos;
        }

        int currentPos = leftSlideMotor.getCurrentPosition();
        slidesRetracted = currentPos < RETRACTED_THRESHOLD;

        slidePower = -opmode.gamepad1.left_stick_y;
        usePID = opmode.gamepad1.left_trigger > 0.1;
        if (usePID) {
            output = controller.calculate(currentPos, target);
            output = (target < currentPos ? -1 : 1) * Math.sqrt(Math.abs(output)) + (slidesRetracted && target < RETRACTED_THRESHOLD ? 0: Kg);
            leftSlideMotor.setPower(output);
            rightSlideMotor.setPower(output);
        } else {
            // manual
            leftSlideMotor.setPower(slidePower);
            rightSlideMotor.setPower(slidePower);
        }


        // updates boolean
        opmode.telemetry.addData("current pos: ", currentPos);
        opmode.telemetry.addData("target: ", target);
        opmode.telemetry.addData("left current: ", leftSlideMotor.getCurrent(CurrentUnit.AMPS));
        opmode.telemetry.addData("right current: ", rightSlideMotor.getCurrent(CurrentUnit.AMPS));
        opmode.telemetry.addData("use PID: ", usePID);
        opmode.telemetry.addData("slidesRetracted: ", slidesRetracted);
        opmode.telemetry.addData("calculated output", output);

        packet.put("current pos: ", currentPos);
        packet.put("target: ", target);
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
    public void raiseToPrepClip()   { moveToPosition(prepClipPos);}
    public void retract()           { moveToPosition(retractedPos);}
    public void raiseToPickupClip() { moveToPosition(pickupClipPos);}

    private boolean isDifferent(double val1, double val2) {
        return Math.abs(val1 - val2) >= CACHING_THRESHOLD;
    }
}


package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class HorizontalSlides {
    OpMode opmode;
    private PIDController controller;
    private DcMotorEx slideMotor;

    // constants
    public static double Kp = 0.005;
    public static double Ki = 0;
    public static double Kd = 0.00001;
    public static double Kg = 0; // no need
    public static double CACHING_THRESHOLD = 0.005;
    public static double RETRACTED_THRESHOLD = 10;
    public static int UPPER_LIMIT = 1000; // this is for 1150s
    public static int LOWER_LIMIT = -2;
    public static double MAPPING_EXPONENT = 1;

    // encoder positions
    public static int extendedPos = 800;
    public static int halfExtendedPos = 500;
    public static int retractedPos = 0;

    // declaring variables for later modification
    private volatile double target = 0;
    private volatile double slidePower;
    private volatile double output = 0;
    private volatile double previousOutput = 0;
    public volatile boolean slidesRetracted = true;
    public volatile boolean usePID = true;

    public HorizontalSlides() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware, boolean resetEncoders) {
        this.opmode = opmode;
        slideMotor = robotHardware.horiMotor;

        controller = new PIDController(Kp, Ki, Kd);

        if (resetEncoders) {
            slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void operate() {
        target = mapToTarget(opmode.gamepad1.right_trigger);
        int currentPos = slideMotor.getCurrentPosition();
        slidesRetracted = currentPos < RETRACTED_THRESHOLD;
        output = controller.calculate(currentPos, target);
        output = Math.max(-1, Math.min(1, output));

        // includes power caching
        if (isDifferent(output, previousOutput)) {
            slideMotor.setPower(output);
        }
        previousOutput = output;
    }

    public void operateTuning(TelemetryPacket packet) {
        if (opmode.gamepad1.y) {
            target = extendedPos;
        } else if (opmode.gamepad1.a) {
            target = retractedPos;
        }

        int currentPos = slideMotor.getCurrentPosition();

        slidePower = -opmode.gamepad1.right_stick_y * 0.5;
        usePID = opmode.gamepad1.left_trigger > 0.1;
        if (usePID) {
            output = controller.calculate(currentPos, target);
            slideMotor.setPower(output);
        } else {
            // manual control
            slideMotor.setPower(slidePower);
        }

        // updates boolean
        slidesRetracted = currentPos < RETRACTED_THRESHOLD;

        // telemetry
        opmode.telemetry.addData("current pos: ", currentPos);
        opmode.telemetry.addData("target: ", target);
        opmode.telemetry.addData("use PID: ", usePID);
        opmode.telemetry.addData("slidesRetracted: ", slidesRetracted);

        packet.put("current pos: ", currentPos);
        packet.put("target: ", target);
    }


    public void operateFix() {
        // manual control
        slidePower = -opmode.gamepad2.right_stick_y;

        if (Math.abs(slidePower) > 0.05) {
            slideMotor.setPower(slidePower);
        }

        if (opmode.gamepad2.left_stick_button) {
            slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

    }

    public void moveToPosition(int targetPos) {
        target = targetPos;
    }

    public void extend()        { moveToPosition(extendedPos); }
    public void extendHalfway() { moveToPosition(halfExtendedPos); }
    public void retract()       { moveToPosition(retractedPos); }

    public double telemetryMotorPos() { return slideMotor.getCurrentPosition(); }
    public double telemetryTarget() { return target; }
    public double telemetryOutput() { return output; }


    public int mapToTarget(double input) {
        return (int) Math.round(Math.pow(input, MAPPING_EXPONENT) * extendedPos);
        // paste this into desmos to see graph: x^{0.4}\ \left\{0\le x\le1\right\}
        // making the mapping exponent smaller makes the graph steeper
    }

    private boolean isDifferent(double val1, double val2) {
        return Math.abs(val1 - val2) >= CACHING_THRESHOLD;
    }
}



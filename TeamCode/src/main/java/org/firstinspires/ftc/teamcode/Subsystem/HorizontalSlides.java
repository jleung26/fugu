package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class HorizontalSlides {
    OpMode opmode;
    private PIDController controller;
    public DcMotorEx horiMotor;

    // constants
    public static double Kp = 0.005;
    public static double Ki = 0;
    public static double Kd = 0.00001;
    public static double Kg = 0; // no need
    public static double CACHING_THRESHOLD = 0.005;
    public static double RETRACTED_THRESHOLD = 10;
    public static int UPPER_LIMIT = 1100; // this is for 1150s
    public static int LOWER_LIMIT = -2;
    public static double MAPPING_EXPONENT = 1;

    // encoder positions
    public static int extendedPos = 1000;
    public static int halfExtendedPos = 500;
    public static int retractedPos = 0;

    // declaring variables for later modification
    private volatile double target = 0;
    private volatile double slidePower = 0;
    private volatile double output = 0;
    private volatile double previousOutput = 0;
    public volatile boolean slidesRetracted = true;
    public volatile boolean usePID = false;

    public HorizontalSlides() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware, boolean resetEncoders) {
        this.opmode = opmode;
        this.horiMotor = robotHardware.horiMotor;

        horiMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horiMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        horiMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(Kp, Ki, Kd);

        if (resetEncoders) {
            horiMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void operate() {
        target = mapToTarget(opmode.gamepad1.right_trigger);
        int currentPos = horiMotor.getCurrentPosition();
        slidesRetracted = currentPos < RETRACTED_THRESHOLD;
        output = controller.calculate(currentPos, target);
        output = Math.max(-1, Math.min(1, output));

        // includes power caching
        if (isDifferent(output, previousOutput)) {
            horiMotor.setPower(output);
        }
        previousOutput = output;
    }

    public void operateTuning() {
        horiMotor.setPower(-opmode.gamepad1.left_stick_y);
//        if (opmode.gamepad1.y) {
//            target = extendedPos;
//        } else if (opmode.gamepad1.a) {
//            target = retractedPos;
//        }

        int currentPos = horiMotor.getCurrentPosition();
//        usePID = opmode.gamepad1.left_trigger > 0.1;
//        if (usePID) {
//            output = controller.calculate(currentPos, target);
//            horiMotor.setPower(output);
//        }
//        else {
//            // manual control
//        horiMotor.setPower(-opmode.gamepad1.left_stick_y);
//        }
//        if (opmode.gamepad1.y) {
//            horiMotor.setPower(0.4);
//        } else if (opmode.gamepad1.a) {
//            horiMotor.setPower(0);
//        }

        // updates boolean
        slidesRetracted = currentPos < RETRACTED_THRESHOLD;

        // telemetry
        opmode.telemetry.addData("current pos: ", currentPos);
        opmode.telemetry.addData("enabled?: ", horiMotor.isMotorEnabled());
        opmode.telemetry.addData("current: ", horiMotor.getCurrent(CurrentUnit.AMPS));
        opmode.telemetry.addData("target: ", target);
        opmode.telemetry.addData("use PID: ", usePID);
        opmode.telemetry.addData("slidesRetracted: ", slidesRetracted);
        opmode.telemetry.addData("left stick y", -opmode.gamepad1.left_stick_y);

//        packet.put("current pos: ", currentPos);
//        packet.put("target: ", target);
    }


    public void operateFix() {
        // manual control
        slidePower = -opmode.gamepad2.right_stick_y;

        if (Math.abs(slidePower) > 0.05) {
            horiMotor.setPower(slidePower);
        }

        if (opmode.gamepad2.left_stick_button) {
            horiMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

    }

    public void moveToPosition(int targetPos) {
        target = targetPos;
    }

    public void extend()        { moveToPosition(extendedPos); }
    public void extendHalfway() { moveToPosition(halfExtendedPos); }
    public void retract()       { moveToPosition(retractedPos); }

    public double telemetryMotorPos() { return horiMotor.getCurrentPosition(); }
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



package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class Hang {
    private OpMode opmode;
    private Servo leftWheely, rightWheely, leftPto, rightPto;
//    private Servo armAngleServo, armPitchServo;
    private DcMotorEx Fl, Fr, Bl, Br, leftVertMotor, rightVertMotor;
    private ElapsedTime elapsedTime;
    private int hangState;

    public static double leftWheelyStowPos = 0.7928;
    public static double leftWheelyDeployPos = 0.097;
    public static double rightWheelyStowPos = 0.8661;
    public static double rightWheelyDeployPos = 0.147;
    public static double leftPtoStowPos = 0.9639;
    public static double leftPtoDeployPos = 0.7617;
    public static double rightPtoStowPos = 1;
    public static double rightPtoDeployPos = 0.8078;


    public Hang() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        this.leftWheely = robotHardware.leftWheelyServo;
        this.rightWheely = robotHardware.rightWheelyServo;
        this.leftPto = robotHardware.leftPtoServo;
        this.rightPto = robotHardware.rightPtoServo;
        this.leftVertMotor = robotHardware.leftVertMotor;
        this.rightVertMotor = robotHardware.rightVertMotor;
//        this.armAngleServo = robotHardware.armAngleServo;
//        this.armPitchServo = robotHardware.armPitchServo;
        this.Fl = robotHardware.Fl;
        this.Fr = robotHardware.Fr;
        this.Bl = robotHardware.Bl;
        this.Br = robotHardware.Br;
        Fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void operateManualTesting() {
        if (opmode.gamepad1.dpad_up) {
            incremental(leftWheely,1);
        } else if (opmode.gamepad1.dpad_down) {
            incremental(leftWheely, -1);
        }

        if (opmode.gamepad1.dpad_left) {
            incremental(rightWheely,1);
        } else if (opmode.gamepad1.dpad_right) {
            incremental(rightWheely, -1);
        }

        if (opmode.gamepad1.y) {
            incremental(leftPto,1);
        } else if (opmode.gamepad1.a) {
            incremental(leftPto, -1);
        }

        if (opmode.gamepad1.x) {
            incremental(rightPto,1);
        } else if (opmode.gamepad1.b) {
            incremental(rightPto, -1);
        }

        double leftStick = -opmode.gamepad1.left_stick_y;
        double rightStick = -opmode.gamepad1.right_stick_y;
        if (Math.abs(leftStick) > 0.05) {
            // only slides
            leftVertMotor.setPower(leftStick);
            rightVertMotor.setPower(leftStick);
        } else if (Math.abs(rightStick) > 0.05) {
//            leftVertMotor.setPower(rightStick);
//            rightVertMotor.setPower(rightStick);
            Fl.setPower(-rightStick);
            Fr.setPower(-rightStick);
            Bl.setPower(-rightStick);
            Br.setPower(-rightStick);
        }

        opmode.telemetry.addData(leftWheely + " pos: ", leftWheely.getPosition());
        opmode.telemetry.addData(rightWheely + " pos: ", rightWheely.getPosition());
        opmode.telemetry.addData(leftPto + " pos: ", leftPto.getPosition());
        opmode.telemetry.addData(rightPto + " pos: ", rightPto.getPosition());
    }

    public void operateSetPos() {
        if (opmode.gamepad1.a) {
            leftWheely.setPosition(leftWheelyDeployPos);
            rightWheely.setPosition(rightWheelyDeployPos);
        } else if (opmode.gamepad1.y) {
            leftWheely.setPosition(leftWheelyStowPos);
            rightWheely.setPosition(rightWheelyStowPos);
        }

        if (opmode.gamepad1.dpad_down) {
            leftPto.setPosition(leftPtoDeployPos);
            rightPto.setPosition(rightPtoDeployPos);
        } else if (opmode.gamepad1.dpad_up) {
            leftPto.setPosition(leftPtoStowPos);
            rightPto.setPosition(rightPtoStowPos);
        }

        double leftStick = -opmode.gamepad1.left_stick_y;
        double rightStick = -opmode.gamepad1.right_stick_y;
        if (Math.abs(leftStick) > 0.05) {
            // only slides
            leftVertMotor.setPower(leftStick);
            rightVertMotor.setPower(leftStick);
        } else if (Math.abs(rightStick) > 0.05) {
//            leftVertMotor.setPower(rightStick);
//            rightVertMotor.setPower(rightStick);
            Fl.setPower(-rightStick);
            Fr.setPower(-rightStick);
            Bl.setPower(-rightStick);
            Br.setPower(-rightStick);
        } else {
            leftVertMotor.setPower(leftStick);
            rightVertMotor.setPower(leftStick);
            Fl.setPower(-opmode.gamepad1.right_trigger);
            Fr.setPower(-opmode.gamepad1.right_trigger);
            Bl.setPower(-opmode.gamepad1.right_trigger);
            Br.setPower(-opmode.gamepad1.right_trigger);
        }

    }

    // I am so proud of this. This is a huge improvement compared to what I used to do.
    public void incremental(Servo servo, int sign) {
        servo.setPosition(servo.getPosition() + sign * 0.001);
    }

//    public void switchToHangMode() {
//        leftVertMotor.setTargetPosition(0);
//        rightVertMotor.setTargetPosition(0);
//        leftVertMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightVertMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftVertMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightVertMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        Fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        // drive and vert slide operates need to be turned off too
//    }
//
//    public void operate() {
//        opmode.telemetry.addData("hangState: ", hangState);
//    }
//
//    public void updateSequenceState() {
//        switch (hangState) {
//            case 0:
//                leftWheely.setPosition(0);
//                rightWheely.setPosition(0);
//                setHangState(1);
//                break;
//            case 1:
//                leftVertMotor.setTargetPosition(0);
//                if (!leftVertMotor.isBusy()) {
//                    setHangState(2);
//                }
//            // and etc. but i don't wanna do that rn
//        }
//    }
//
//    public void setHangState(int x) {
//        hangState = x;
//    }


    // drive up
    // sequence begins:
    // slides extend 1/2 and wheely goes
    // PTO engage
    // drive motors go
    // swinging
    // slides extend tiny bit to transfer load to passive hooks
    // PTO disengage
    // slides extend fully
    // delay for balance, or wait for driver confirmation, or arm swing over, or gyro, or combo of prev
    // PTO re-engage, wheely retracts
    // pull until encoder detect
    // power all to 0 (or all to 0.2 stall?)

}

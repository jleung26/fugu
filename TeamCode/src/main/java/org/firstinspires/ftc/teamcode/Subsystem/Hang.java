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
    private Servo leftWheely, rightWheely, leftPtoServo, rightPtoServo;
//    private Servo armAngleServo, armPitchServo;
    private DcMotorEx Fl, Fr, Bl, Br, leftVertMotor, rightVertMotor;
    private ElapsedTime elapsedTime;
    private int hangState;

    public static double leftWheelyStowPos = 0;
    public static double leftWheelyDeployPos = 0;
    public static double rightWheelyStowPos = 0;
    public static double rightWheelyDeployPos = 0;
    public static double leftPtoStowPos = 0;
    public static double leftPtoDeployPos = 0;
    public static double rightPtoStowPos = 0;
    public static double rightPtoDeployPos = 0;


    public Hang() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        this.leftWheely = robotHardware.leftWheelyServo;
        this.rightWheely = robotHardware.rightWheelyServo;
        this.leftPtoServo = robotHardware.leftPtoServo;
        this.rightPtoServo = robotHardware.rightPtoServo;
        this.leftVertMotor = robotHardware.leftVertMotor;
        this.rightVertMotor = robotHardware.rightVertMotor;
//        this.armAngleServo = robotHardware.armAngleServo;
//        this.armPitchServo = robotHardware.armPitchServo;

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
            incremental(leftPtoServo,1);
        } else if (opmode.gamepad1.a) {
            incremental(rightPtoServo, -1);
        }

        if (opmode.gamepad1.x) {
            incremental(leftPtoServo,1);
        } else if (opmode.gamepad1.b) {
            incremental(rightPtoServo, -1);
        }

        double leftStick = -opmode.gamepad1.left_stick_y;
        double rightStick = -opmode.gamepad1.right_stick_y;
        if (Math.abs(leftStick) > 0.05) {
            // only slides
            leftVertMotor.setPower(leftStick);
            rightVertMotor.setPower(leftStick);
        } else if (Math.abs(rightStick) > 0.05) {
            leftVertMotor.setPower(rightStick);
            rightVertMotor.setPower(rightStick);
            Fl.setPower(-rightStick);
            Fr.setPower(-rightStick);
            Bl.setPower(-rightStick);
            Br.setPower(-rightStick);
        }

        opmode.telemetry.addData(leftWheely + " pos: ", leftWheely.getPosition());
        opmode.telemetry.addData(rightWheely + " pos: ", rightWheely.getPosition());
        opmode.telemetry.addData(leftPtoServo + " pos: ", leftPtoServo.getPosition());
        opmode.telemetry.addData(rightPtoServo + " pos: ", rightPtoServo.getPosition());
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
    // all motors go
    // swinging
    // slides extend tiny bit to transfer load to passive hooks
    // PTO disengage
    // slides extend fully
    // delay for balance, or wait for driver confirmation, or arm swing over, or gyro, or combo of prev
    // PTO re-engage, wheely retracts
    // pull until encoder detect
    // power all to 0 (or all to 0.2 stall?)

}

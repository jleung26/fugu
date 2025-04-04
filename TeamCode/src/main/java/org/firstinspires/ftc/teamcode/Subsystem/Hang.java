package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class Hang {
    private OpMode opmode;
    private Servo leftWheely, rightWheely, leftPto, rightPto;
    private Servo rightArmPitchServo, leftArmPitchServo, armExtenderServo;
    private DcMotorEx Fl, Fr, Bl, Br, leftVertMotor, rightVertMotor;
    private int hangState;

    public static double leftWheelyStowPos = 0.7928;
    public static double leftWheelyDeployPos = 0.097;
    public static double rightWheelyStowPos = 0.8661;
    public static double rightWheelyDeployPos = 0.147;
    public static double leftPtoStowPos = 0.9639;
    public static double leftPtoDeployPos = 0.7617;
    public static double rightPtoStowPos = 1;
    public static double rightPtoDeployPos = 0.8078;
    public static double armL2Pos = 0;
    public static double armSwingPos = 0;
    public static double armExtenderRetractedPos = 0;
    public static double armExtenderExtendedPos = 0;


    public Hang() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        this.leftWheely = robotHardware.leftWheelyServo;
        this.rightWheely = robotHardware.rightWheelyServo;
        this.leftPto = robotHardware.leftPtoServo;
        this.rightPto = robotHardware.rightPtoServo;
        this.leftVertMotor = robotHardware.leftVertMotor;
        this.rightVertMotor = robotHardware.rightVertMotor;
//        this.rightArmPitchServo = robotHardware.rightArmPitchServo;
//        this.leftArmPitchServo = robotHardware.leftArmPitchServo;
//        this.armExtenderServo = robotHardware.armExtenderServo;
        this.Fl = robotHardware.Fl;
        this.Fr = robotHardware.Fr;
        this.Bl = robotHardware.Bl;
        this.Br = robotHardware.Br;
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

    public void switchToHangMode() {
        leftVertMotor.setTargetPosition(0);
        rightVertMotor.setTargetPosition(0);
        leftVertMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVertMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftVertMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightVertMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // TODO: drive and vert slide operates need to be turned off
    }

    public void operate(Gamepad currentGamepad1, Gamepad currentGamepad2, Gamepad previousGamepad1, Gamepad previousGamepad2) {
        updateSequenceState(currentGamepad1,currentGamepad2, previousGamepad1, previousGamepad2);

        opmode.telemetry.addData("hangState: ", hangState);
    }

    public void updateSequenceState(Gamepad currentGamepad1, Gamepad currentGamepad2, Gamepad previousGamepad1, Gamepad previousGamepad2) {
        switch (hangState) {
            case 0: /// set motor modes, arm pos, and start wheely (the servos strain a lot, so move quickly)
                if (currentGamepad2.a && !previousGamepad2.a) {
                    // motors all in modes
                    switchToHangMode();
                    // raise slides to 2nd bar
                    leftVertMotor.setTargetPosition(500);
                    rightVertMotor.setTargetPosition(500);
                    // wheely
                    deployWheely();
                    // arm gets out of the way
                    leftArmPitchServo.setPosition(armL2Pos);
                    rightArmPitchServo.setPosition(armL2Pos);
                    armExtenderServo.setPosition(armExtenderRetractedPos);
                    setHangState(1);
                }
                break;
            /// driver 1 positions bot, driver 2 uses left stick to straight drive and tip (need good timing)
            case 1: /// engage PTO, disable slide motors
                if (currentGamepad2.a && !previousGamepad2.a) {
                    // stop unnecessary current draw
                    leftVertMotor.setMotorDisable(); // never used before, could be problematic, but seems self-explanatory
                    rightVertMotor.setMotorDisable();
                    // engage PTO
                    leftPto.setPosition(leftPtoDeployPos);
                    rightPto.setPosition(rightPtoDeployPos);
                    setHangState(2);
                }
                break;
            /// driver 2 uses left stick manual control (operate method in mecanum file) to pull up
            /// driver 2 uses left stick manual control (in mecanum file) to drop onto secondary hooks
            /// driver 2 uses left stick manual control to extend to L3 height
            case 2: /// arm swings over, while driver 2 times to land hooks
                if (currentGamepad2.a && !previousGamepad2.a) {
                    leftArmPitchServo.setPosition(armSwingPos);
                    rightArmPitchServo.setPosition(armSwingPos);
                    armExtenderServo.setPosition(armExtenderExtendedPos); // might need modification, idk where the linkage will intersect
                    setHangState(-1);
                }
                break;
            /// driver 2 does final pull up and puts controller down
        }
    }

    public void setHangState(int x) {
        hangState = x;
    }

    public void deployWheely() {
        leftWheely.setPosition(leftWheelyDeployPos);
        rightWheely.setPosition(rightWheelyDeployPos);
    }
    public void stowWheely() {
        leftWheely.setPosition(leftWheelyStowPos);
        rightWheely.setPosition(rightWheelyStowPos);
    }

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

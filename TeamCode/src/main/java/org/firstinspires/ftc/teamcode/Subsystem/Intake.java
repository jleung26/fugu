package org.firstinspires.ftc.teamcode.Subsystem;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.Locale;

@Config
public class Intake {
    OpMode opmode;
    public DcMotorEx intakeMotor;
    public Servo leftWrist, rightWrist;
    public RevColorSensorV3 colorSensor;

    public enum IntakeChamberState {
        BLUE,
        RED,
        YELLOW,
        EMPTY,
        UNKNOWN
    }

    public enum IntakeState {
        INTAKING,
        REVERSE,
        NEUTRAL
    }

    public volatile IntakeChamberState chamberState = IntakeChamberState.EMPTY;
    public volatile IntakeState intakeState = IntakeState.NEUTRAL;

    // motor constants
    private double INTAKING_POWER = 1;
    private double REVERSE_POWER = -1;
    private double NEUTRAL_POWER = 0;

    // wrist constants // wrists synchronized :)
    private double TRANSFER_POS = 0.08;
    private double DROP_DOWN_POS = 0.416;
    private double WRIST_TUNING_INCREMENT = 0.001;

    // sensor constants
    public static double DETECTION_THRESHOLD = 1.2; // inches
    public static double BLUE_RGB_THRESHOLD = 420;
    public static double YELLOW_RGB_THRESHOLD = 500;

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        intakeMotor = robotHardware.intakeMotor;
        leftWrist = robotHardware.leftIntakeWristServo;
        rightWrist = robotHardware.rightIntakeWristServo;
        colorSensor = robotHardware.intakeColorSensor;
    }

    public void operateTesting() {
        intakeMotor.setPower(-opmode.gamepad1.left_stick_y);

        if (opmode.gamepad1.left_trigger > 0.5) {
            incremental(leftWrist, 1);
            incremental(rightWrist, 1);
        } else if (opmode.gamepad1.right_trigger > 0.5) {
            incremental(leftWrist, -1);
            incremental(rightWrist, -1);
        }

        if (opmode.gamepad1.a) {
            intake();
        } else if (opmode.gamepad1.b) {
            neutral();
        } else if (opmode.gamepad1.y) {
            reverse();
        }

        if (opmode.gamepad1.dpad_up) {
            flipUp();
        } else if (opmode.gamepad1.dpad_down) {
            dropDown();
        }

        chamberState = getPieceColor();

//        opmode.telemetry.addLine(String.format(Locale.US, "Red %d, Green %d, Blue %d", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
//        opmode.telemetry.addData("distance detected", colorSensor.getDistance(DistanceUnit.INCH));
        opmode.telemetry.addData("chamber color enum: ", chamberState);
        opmode.telemetry.addData("intake state enum: ", intakeState);
        opmode.telemetry.addData("left wrist pos: ", leftWrist.getPosition());
        opmode.telemetry.addData("right wrist pos: ", rightWrist.getPosition());

    }

    public void operateTeleOp() {}

    public void operateColorChecking(boolean rejectBlue) {
        chamberState = getPieceColor();

        // if empty, start intaking (only once instead of once every loop)
        if (chamberState == IntakeChamberState.EMPTY && intakeState != IntakeState.INTAKING) {
            intake();
            // if chamber full, check color and reverse once, and another statement will eventually set back to intaking
        } else if (chamberState != IntakeChamberState.EMPTY) {
            if (chamberState == (rejectBlue ? IntakeChamberState.BLUE : IntakeChamberState.RED) && intakeState != IntakeState.REVERSE) {
                reverse();
            } else if ((chamberState == (rejectBlue ? IntakeChamberState.RED : IntakeChamberState.BLUE) || chamberState == IntakeChamberState.YELLOW) && intakeState != IntakeState.NEUTRAL) {
                neutral();
            }
        }

        opmode.telemetry.addData("chamber color enum: ", chamberState);
        opmode.telemetry.addData("intake state enum: ", intakeState);
    }

    // intake method
    private void setIntake(double power) {intakeMotor.setPower(power);}
    public void intake() {
        setIntake(INTAKING_POWER);
//        dropDown(); separate for now, because idk how we'll use with actions
        intakeState = IntakeState.INTAKING;
    }
    public void reverse() {
        setIntake(REVERSE_POWER);
        intakeState = IntakeState.REVERSE;
    }
    public void neutral() {
        setIntake(NEUTRAL_POWER);
        intakeState = IntakeState.NEUTRAL;
    }

    // wrist methods
    public void incremental(Servo servo, int sign) {servo.setPosition(servo.getPosition() + sign * WRIST_TUNING_INCREMENT);}
    public void dropDown() { leftWrist.setPosition(DROP_DOWN_POS); rightWrist.setPosition(DROP_DOWN_POS);}
    public void flipUp() { leftWrist.setPosition(TRANSFER_POS); rightWrist.setPosition(TRANSFER_POS);}

    public IntakeChamberState getPieceColor() {
        if (colorSensor.getDistance(DistanceUnit.INCH) < DETECTION_THRESHOLD) {
            double blue = colorSensor.blue();
//            double red = colorSensor.red();
            double green = colorSensor.green();

            // Minimal comparisons, breaks if the reflected light is too saturated and nearly white
            if (blue > BLUE_RGB_THRESHOLD) {
                return IntakeChamberState.BLUE; // Blue is dominant -> blue
            } else if (green > YELLOW_RGB_THRESHOLD) {
                return IntakeChamberState.YELLOW; // Green dominant, blue not dominant -> (Yellow)
            } else {
                return IntakeChamberState.RED; // Not blue or yellow -> red
            }
        } else {
            return IntakeChamberState.EMPTY;
        }
    }
    // last resort, but first draft already works
//    public IntakeChamberState getColorViaHSV() {
//        if (colorSensor.getDistance(DistanceUnit.INCH) < DETECTION_THRESHOLD) {
//            float[] hsvValues = new float[3];
//            Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
//            float hue = hsvValues[0];
//            if (hue >= 190 && hue < 260) {
//                return IntakeChamberState.BLUE;
//            } else if (hue >= 30 && hue < 90) {
//                return IntakeChamberState.YELLOW;
//            } else if (hue < 30 || hue > 260) {
//                return IntakeChamberState.RED;
//            } else {
//                return IntakeChamberState.UNKNOWN;
//            }
//        } else {
//            return IntakeChamberState.EMPTY;
//        }
//    }
}
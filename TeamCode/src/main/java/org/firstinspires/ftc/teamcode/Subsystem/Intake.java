package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

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
        IDLE
    }

    // motor constants
    public static double INTAKING_POWER = 0.6;
    public static double REVERSE_POWER = -0.6;
    public static double IDLE_POWER = 0.1;

    // wrist constants // wrists synchronized :)
    public static double TRANSFER_POS = 0.087;
    public static double DROP_DOWN_POS = 0.39;
    public static double PARTIAL_DROPDOWN_POS = 0.217;
    public static double WRIST_TUNING_INCREMENT = 0.001;

    // sensor constants
    public static double DETECTION_THRESHOLD = 1.2; // inches

    // constantly updating states
    public volatile IntakeChamberState chamberState = IntakeChamberState.EMPTY;
    public volatile IntakeChamberState prevChamberState = IntakeChamberState.EMPTY;
    public volatile IntakeState intakeState = IntakeState.IDLE;
    public volatile boolean wristFlippedUp = true;

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
            idle();
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

    public void operateColorChecking() {
        prevChamberState = chamberState;
        chamberState = getPieceColor();

        opmode.telemetry.addData("chamber color enum: ", chamberState);
//        opmode.telemetry.addData("intake state enum: ", intakeState);
//        opmode.telemetry.addData("argb: ", colorSensor.argb());
//        opmode.telemetry.addLine(String.format(Locale.US, "Red %d, Green %d, Blue %d", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
    }

    // intake method
    public void setIntake(double power) {intakeMotor.setPower(power);}
    public void intake() {
        setIntake(INTAKING_POWER);
        intakeState = IntakeState.INTAKING;
    }
    public void reverse() {
        setIntake(REVERSE_POWER);
        intakeState = IntakeState.REVERSE;
    }
    public void idle() {
        setIntake(IDLE_POWER);
        intakeState = IntakeState.IDLE;
    }
    public void fullStop() {
        setIntake(0);
        intakeState = IntakeState.IDLE;
    }

    // wrist methods
    public void incremental(Servo servo, int sign) {servo.setPosition(servo.getPosition() + sign * WRIST_TUNING_INCREMENT);}
    public void dropDown() { leftWrist.setPosition(DROP_DOWN_POS); rightWrist.setPosition(DROP_DOWN_POS); wristFlippedUp = false;}
    public void flipUp() { leftWrist.setPosition(TRANSFER_POS); rightWrist.setPosition(TRANSFER_POS); wristFlippedUp = true;}
    public void partialDropDown() {leftWrist.setPosition(PARTIAL_DROPDOWN_POS); rightWrist.setPosition(PARTIAL_DROPDOWN_POS); wristFlippedUp = false;}

    public IntakeChamberState getPieceColor() {
        if (colorSensor.getDistance(DistanceUnit.INCH) < DETECTION_THRESHOLD) {
            int argb = colorSensor.argb();
            int red   = (argb >> 16) & 0xFF; // Extract red
            int green = (argb >> 8)  & 0xFF; // Extract green
            int blue  = argb & 0xFF;         // Extract blue
//            double blue = colorSensor.blue();
//            double red = colorSensor.red();
//            double green = colorSensor.green();

            // Minimal comparisons, breaks if the reflected light is too saturated and nearly white
//            if (blue > BLUE_RGB_THRESHOLD) {
//                return IntakeChamberState.BLUE; // Blue is dominant -> blue
//            } else if (green > YELLOW_RGB_THRESHOLD) {
//                return IntakeChamberState.YELLOW; // Green dominant, blue not dominant -> (Yellow)
//            } else {
//                return IntakeChamberState.RED; // Not blue or yellow -> red
//            }

            if (blue > red && blue > green) {
                return IntakeChamberState.BLUE; // Blue is dominant -> blue
            } else if (red > blue && red > green) {
                return IntakeChamberState.RED; // Not blue or yellow -> red
            } else if (green > red && green > blue) {
                return IntakeChamberState.YELLOW; // Green dominant, blue not dominant -> (Yellow)
            } else {
                return IntakeChamberState.UNKNOWN; // Handle ambiguous cases
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
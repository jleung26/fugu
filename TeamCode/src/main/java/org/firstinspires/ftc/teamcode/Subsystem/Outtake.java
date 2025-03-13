package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class Outtake {

    OpMode opmode;
    public Claw claw = new Claw();
    public Arm arm = new Arm();
    public Wrist wrist = new Wrist();
    private final RobotHardware rHardware = new RobotHardware();

    public Outtake() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;

        claw.initialize(robotHardware);
        arm.initialize(robotHardware);
        wrist.initialize(robotHardware);
    }

    public void operateTuning() {
        // tuning
        if (opmode.gamepad1.left_bumper) {
            claw.incremental(-1);
        }
        else if (opmode.gamepad1.right_bumper) {
            claw.incremental(1);
        }

        wrist.incremental(getSign(-opmode.gamepad1.left_stick_y));

        if (opmode.gamepad1.dpad_up) {
            arm.incrementalArm(-1);
        } else if (opmode.gamepad1.dpad_down) {
            arm.incrementalArm(1);
        }

        // testing full movements
        if (opmode.gamepad2.a) { // transfer
            wrist.setWristTransfer();
            arm.setArmTransfer();
        }
        else if (opmode.gamepad2.b) { // stow
            wrist.setWristStow();
            arm.setArmStow();
        }
        else if (opmode.gamepad2.y) { // scoring bucket
            wrist.setWristScoreBucket();
            arm.setArmScoreBucket();
        } else if (opmode.gamepad2.x) { // scoring clip
            wrist.setWristScoreClip();
            arm.setArmScoreClip();
        }

        if (opmode.gamepad2.dpad_up) {
            claw.closeClawTight();
        } else if (opmode.gamepad2.dpad_left) {
            claw.closeClawLoose();
        } else if (opmode.gamepad2.dpad_down) {
            claw.openClaw();
        }

        opmode.telemetry.addData("Scoring Arm Pos: ", arm.arm.getPosition());
        opmode.telemetry.addData("Scoring Wrist Pos: ", wrist.wrist.getPosition());
        opmode.telemetry.addData("Scoring Claw Pos: ", claw.claw.getPosition());
        opmode.telemetry.update();
    }

    public static class Arm {
        public Servo arm;
        public enum STATE {
            STOW,
            TRANSFER,
            SCORING_BUCKET,
            SCORING_CLIP,
            GRABBING_CLIP
        }

        public Arm.STATE armPos = STATE.STOW;
        public static double armStowPosition = 0;
        public static double armTransferPosition = 0;
        public static double armScoringBucketPosition = 0;
        public static double armScoringClipPosition = 0;
        public static double armGrabClipWallPosition = 0;
        public static double armIncrement = 0.0005;

        public Arm() {}

        public void initialize(RobotHardware robotHardware) {
            this.arm = robotHardware.armAngleServo;
        }

        public void setArmPosition(double position) {
            arm.setPosition(position);
        }

        // Incremental arm movement function
        public void incrementalArm(int sign) {
            arm.setPosition(arm.getPosition() + sign * armIncrement);
        }

        public void setArmScoreBucket() {
            setArmPosition(armScoringBucketPosition);
            armPos = STATE.SCORING_BUCKET;
        }
        public void setArmScoreClip() {
            setArmPosition(armScoringClipPosition);
            armPos = STATE.SCORING_CLIP;
        }
        public void setArmGrabClip() {
            setArmPosition(armGrabClipWallPosition);
            armPos = STATE.GRABBING_CLIP;
        }
        public void setArmTransfer() {
            setArmPosition(armTransferPosition);
            armPos = STATE.TRANSFER;
        }
        public void setArmStow() {
            setArmPosition(armStowPosition);
            armPos = STATE.STOW;
        }

        public double telemetryArmPos() {
            return arm.getPosition();
        }
    }

    public static class Wrist {
        public Servo wrist;
        public boolean isWristTransferring = true;
        public static double wristStowPosition = 0;
        public static double wristTransferPosition = 0;
        public static double wristScoreBucketPosition = 0;
        public static double wristScoreClipPosition = 0;
        public static double wristGrabClipWallPosition = 0;
        public static double wristIncrement = 0.0005;

        public Wrist() {}

        public void initialize(RobotHardware robotHardware) {
            this.wrist = robotHardware.armPitchServo;
        }

        // Set positions
        public void setWristTransfer() {
            wrist.setPosition(wristTransferPosition);
            isWristTransferring = true;
        }
        public void setWristStow() {
            wrist.setPosition(wristStowPosition);;
            isWristTransferring = true;
        }
        public void setWristScoreBucket() {
            wrist.setPosition(wristScoreBucketPosition);
            isWristTransferring = false;
        }
        public void setWristScoreClip() {
            wrist.setPosition(wristScoreClipPosition);
            isWristTransferring = false;
        }
        public void setWristGrabClip() {
            wrist.setPosition(wristGrabClipWallPosition);
            isWristTransferring = false;
        }

        // Incremental wrist turn
        public void incremental(int sign) {
            wrist.setPosition(wrist.getPosition() + sign * wristIncrement);
        }

        public double telemetryWristPos() {
            return wrist.getPosition();
        }
    }

    public static class Claw {
        public Servo claw;
        public boolean isClawOpen = true;
        private final double clawTightClosedPosition = 0;
        private final double clawLooseClosePosition = 0;
        private final double clawOpenPosition = 0;
        private final double clawIncrement = 0.0003;

        public Claw() {}

        public void initialize(RobotHardware robotHardware) {
            this.claw = robotHardware.armClawServo;
        }

        // Toggles the claw between open and closed positions
        public void toggleClaw() {
            if (isClawOpen) {
                claw.setPosition(clawTightClosedPosition);
                isClawOpen = false;
            } else {
                claw.setPosition(clawOpenPosition);
                isClawOpen = true;
            }
        }

        // set positions
        public void closeClawTight() {
            claw.setPosition(clawTightClosedPosition);
            isClawOpen = false;
        }
        public void closeClawLoose() {
            claw.setPosition(clawLooseClosePosition);
            isClawOpen = false;
        }
        public void openClaw() {
            claw.setPosition(clawOpenPosition);
            isClawOpen = true;
        }

        public void incremental(int sign) {
            claw.setPosition(claw.getPosition() + sign * clawIncrement);
        }

        public double telemetryClawPos() {
            return claw.getPosition();
        }
    }

    // simple util function
    public int getSign(double input) {
        return input > 0 ? 1 : input == 0 ? 0 : -1;
    }
}

package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class ExtendingOuttake {

    OpMode opmode;
    public Claw claw = new Claw();
    public ArmPitch armPitch = new ArmPitch();
    public ArmExtend armExtend = new ArmExtend();

    public ExtendingOuttake() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;

        claw.initialize(robotHardware);
        armPitch.initialize(robotHardware);
        armExtend.initialize(robotHardware);
    }

    public void operateTuning() {
        // tuning
        if (opmode.gamepad1.left_bumper) {
            claw.incremental(-1);
        }
        else if (opmode.gamepad1.right_bumper) {
            claw.incremental(1);
        }

        armExtend.incremental(getSign(-opmode.gamepad1.left_stick_y));

        if (opmode.gamepad1.dpad_up) {
            armPitch.incrementalArm(-1);
        } else if (opmode.gamepad1.dpad_down) {
            armPitch.incrementalArm(1);
        }

        // testing full movements
        if (opmode.gamepad2.a) { // transfer
            armExtend.extendToTransfer();
            armPitch.setArmTransfer();
        } else if (opmode.gamepad2.b) { // stow
            armExtend.extendToStow();
            armPitch.setArmStow();
        } else if (opmode.gamepad2.y) { // scoring bucket
            armExtend.extendToScoreBucket();
            armPitch.setArmScoreBucket();
        }  else if (opmode.gamepad2.x) { // scoring clip
            armExtend.extendToScoreClip();
            armPitch.setArmScoreClip();
        } else if (opmode.gamepad2.dpad_down) {
            armExtend.extendToGrabClip();
            armPitch.setArmGrabClip();
        }

        if (opmode.gamepad2.right_bumper) {
            claw.closeTight();
        } else if (opmode.gamepad2.dpad_left) {
            claw.closeLoose();
        } else if (opmode.gamepad2.left_bumper) {
            claw.open();
        }

        opmode.telemetry.addData("Left Arm Pitch Pos: ", armPitch.armL.getPosition());
        opmode.telemetry.addData("Right Arm Pitch Pos: ", armPitch.armR.getPosition());
        opmode.telemetry.addData("Arm Extend Pos: ", armExtend.extenderServo.getPosition());
        opmode.telemetry.addData("Scoring Claw Pos: ", claw.clawServo.getPosition());
        opmode.telemetry.update();
    }

    public static class ArmPitch {
        public Servo armR, armL;
        public enum STATE {
            STOW,
            TRANSFER,
            SCORING_BUCKET,
            SCORING_CLIP,
            GRABBING_CLIP
        }

        public ArmPitch.STATE armPos = STATE.STOW;
        public static double armStowPosition = 0.4317;
        public static double armTransferPosition = 1;
        public static double armScoringBucketPosition = 0.2206;
        public static double armScoringClipPosition = 0.48;
        public static double armGrabClipWallPosition = 0.0494;
        public static double armPrepClipPosition = 0;
        public static double armIncrement = 0.0005;


        public ArmPitch() {}

        public void initialize(RobotHardware robotHardware) {
            this.armR = robotHardware.rightArmPitchServo;
            this.armL = robotHardware.leftArmPitchServo;
        }

        public void setArmPosition(double position) {
            armL.setPosition(position);
            armR.setPosition(position);
        }

        // Incremental arm movement function
        public void incrementalArm(int sign) {
            armL.setPosition(armL.getPosition() + sign * armIncrement);
            armR.setPosition(armR.getPosition() + sign * armIncrement);
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
        public void setArmPrepClip() {
            setArmPosition(armPrepClipPosition);
            armPos = STATE.SCORING_CLIP;
        }
    }

    public static class ArmExtend {
        public Servo extenderServo;
        public boolean isExtenderTransferring = true;
        public static double extenderStowPosition = 0.4078;
        public static double extenderTransferPosition = 0;
        public static double extenderScoreBucketPosition = 0.9;
        public static double extenderScoreClipPosition = 0.4317;
        public static double extenderGrabClipWallPosition = 0.9872;
        public static double increment = 0.0005;

        public ArmExtend() {}

        public void initialize(RobotHardware robotHardware) {
            this.extenderServo = robotHardware.armExtenderServo;
        }

        // Set positions
        public void extendToTransfer() {
            extenderServo.setPosition(extenderStowPosition);
            isExtenderTransferring = true;
        }
        public void extendToStow() {
            extenderServo.setPosition(extenderTransferPosition);;
            isExtenderTransferring = true;
        }
        public void extendToScoreBucket() {
            extenderServo.setPosition(extenderScoreBucketPosition);
            isExtenderTransferring = false;
        }
        public void extendToScoreClip() {
            extenderServo.setPosition(extenderScoreClipPosition);
            isExtenderTransferring = false;
        }
        public void extendToGrabClip() {
            extenderServo.setPosition(extenderGrabClipWallPosition);
            isExtenderTransferring = false;
        }

        // Incremental extension
        public void incremental(int sign) {
            extenderServo.setPosition(extenderServo.getPosition() + sign * increment);
        }
    }

    public static class Claw {
        public Servo clawServo;
        public boolean isClawOpen = true;
        private final double clawTightClosedPosition = 0.197;
        private final double clawLooseClosePosition = 0.25;
        private final double clawOpenPosition = 0.6339;
        private final double clawIncrement = 0.0003;

        public Claw() {}

        public void initialize(RobotHardware robotHardware) {
            this.clawServo = robotHardware.armClawServo;
        }

        // Toggles the clawServo between open and closed positions
        public void toggleClaw() {
            if (isClawOpen) {
                clawServo.setPosition(clawTightClosedPosition);
                isClawOpen = false;
            } else {
                clawServo.setPosition(clawOpenPosition);
                isClawOpen = true;
            }
        }

        // set positions
        public void closeTight() {
            clawServo.setPosition(clawTightClosedPosition);
            isClawOpen = false;
        }
        public void closeLoose() {
            clawServo.setPosition(clawLooseClosePosition);
            isClawOpen = false;
        }
        public void open() {
            clawServo.setPosition(clawOpenPosition);
            isClawOpen = true;
        }

        public void incremental(int sign) {
            clawServo.setPosition(clawServo.getPosition() + sign * clawIncrement);
        }

        public double telemetryClawPos() {
            return clawServo.getPosition();
        }
    }

    // combined actions
    // tranfser
    public void toTransfer() {
        armExtend.extendToTransfer();
        armPitch.setArmTransfer();
    }
    // stow
    public void toStow() {
        armExtend.extendToStow();
        armPitch.setArmStow();
    }
    // score bucket
    public void toScoreBucket() {
        armExtend.extendToScoreBucket();
        armPitch.setArmScoreBucket();
    }
    // score clip
    public void toScoreClip() {
        armExtend.extendToScoreClip();
        armPitch.setArmScoreClip();
    }
    // prep before scoring clip
    public void toPrepScoreClip() {
        armExtend.extendToScoreClip();
        armPitch.setArmPrepClip();
    }
    // grab clip
    public void toGrabClip() {
        armExtend.extendToGrabClip();
        armPitch.setArmGrabClip();
    }

    public void closeClawTight() { claw.closeTight(); }
    public void closeClawLoose() { claw.closeLoose(); }
    public void openClaw()       { claw.open(); }


    // simple util function
    public int getSign(double input) {
        return input > 0 ? 1 : input == 0 ? 0 : -1;
    }
}

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
        /// tuning (GAMEPAD 1)
        if (opmode.gamepad1.left_bumper) { claw.incremental(-1); } //open hopefully?
        else if (opmode.gamepad1.right_bumper) { claw.incremental(1); }

        if (opmode.gamepad1.a) { armExtend.incremental(1); }
        else if (opmode.gamepad1.y) { armExtend.incremental(-1); } // extend hopefully?

        if (opmode.gamepad1.dpad_up) { armPitch.incrementalArm(-1); }
        else if (opmode.gamepad1.dpad_down) { armPitch.incrementalArm(1); }

        /// testing full movements (GAMEPAD 2)
        if      (opmode.gamepad2.a) { toTransfer();}
        else if (opmode.gamepad2.b) { toStow(); }
        else if (opmode.gamepad2.y) { toScoreBucket(); }
        else if (opmode.gamepad2.x) { toScoreClip(); }
        else if (opmode.gamepad2.dpad_down) { toGrabClip(); }

        if      (opmode.gamepad2.right_bumper) { claw.closeTight(); }
        else if (opmode.gamepad2.dpad_left)    { claw.closeLoose(); }
        else if (opmode.gamepad2.left_bumper)  { claw.open(); }

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
            GRABBING_CLIP,
            VERT
        }

        public ArmPitch.STATE armState = STATE.STOW;
        private final double armStowPosition = 0.025;
        private final double armTransferPosition = 0.025;
        private final double armScoringBucketPosition = 0.6;
        private final double armScoringClipPosition = 0.2;
        private final double armGrabClipWallPosition = 0.89;
        private final double armVertPosition = 0.521;
        private final double armIncrement = 0.0005;


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
            armState = STATE.SCORING_BUCKET;
        }
        public void setArmScoreClip() {
            setArmPosition(armScoringClipPosition);
            armState = STATE.SCORING_CLIP;
        }
        public void setArmGrabClip() {
            setArmPosition(armGrabClipWallPosition);
            armState = STATE.GRABBING_CLIP;
        }
        public void setArmTransfer() {
            setArmPosition(armTransferPosition);
            armState = STATE.TRANSFER;
        }
        public void setArmStow() {
            setArmPosition(armStowPosition);
            armState = STATE.STOW;
        }
        public void setArmVert() {
            setArmPosition(armVertPosition);
            armState = STATE.VERT;
        }
    }

    public static class ArmExtend {
        public Servo extenderServo;
        public boolean isExtenderTransferring = true;
        private final double extenderStowPosition = 0.363;
        private final double extenderTransferPosition = 0.8;
        private final double extenderScoreBucketPosition = 0.98;
        private final double extenderScoreClipPosition = 0.95;
        private final double extenderGrabClipWallPosition = 0.1944;
        private final double increment = 0.001;

        public ArmExtend() {}

        public void initialize(RobotHardware robotHardware) {
            this.extenderServo = robotHardware.armExtenderServo;
        }

        // Set positions
        public void extendToTransfer() {
            extenderServo.setPosition(extenderTransferPosition);
            isExtenderTransferring = true;
        }
        public void extendToStow() {
            extenderServo.setPosition(extenderStowPosition);;
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
        private final double clawTightClosedPosition = 0.2;
        private final double clawLooseClosePosition = 0.22;
        private final double clawOpenPosition = 0.66;
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
    public void toTransfer() {
        armExtend.extendToTransfer();
        armPitch.setArmTransfer();
    }
    public void toStow() {
        armExtend.extendToStow();
        armPitch.setArmStow();
    }
    public void toScoreBucket() {
        armExtend.extendToScoreBucket();
        armPitch.setArmScoreBucket();
    }
    public void toScoreClip() {
        armExtend.extendToScoreClip();
        armPitch.setArmScoreClip();
    }
    public void toGrabClip() {
        armExtend.extendToGrabClip();
        armPitch.setArmGrabClip();
    }
    public void toVert() {
        armExtend.extendToGrabClip();
        armPitch.setArmVert();
    } // to vertical (claw facing straight up) "neutral" position (only really intermediate for bucket scoring)

    public void closeClawTight() { claw.closeTight(); }
    public void closeClawLoose() { claw.closeLoose(); }
    public void openClaw()       { claw.open(); }

    public void retractExtender() {armExtend.extendToStow();} // just for scoring clips


    // simple util function
    public int getSign(double input) {
        return input > 0 ? 1 : input == 0 ? 0 : -1;
    }
}

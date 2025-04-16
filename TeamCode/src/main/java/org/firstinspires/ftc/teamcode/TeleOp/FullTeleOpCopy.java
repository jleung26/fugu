package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Hang;
import org.firstinspires.ftc.teamcode.Subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystem.ExtendingOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Full TeleOp FINAL", group = "A")
public class FullTeleOpCopy extends OpMode {
    // subsystems
    RobotHardware robotHardware = new RobotHardware();
    Mecanum drive = new Mecanum();
    VerticalSlides verticalSlides = new VerticalSlides();
    HorizontalSlides horizontalSlides = new HorizontalSlides();
    ExtendingOuttake outtake = new ExtendingOuttake();
    Intake intake = new Intake();
    Hang hang = new Hang();

    // booleans
    // team color
    // spec/sample
    boolean redAllianceBool = true;
    boolean sampleModeBool = true;
    boolean hangBool = false;
    boolean highBucketBool = true;
    Intake.IntakeChamberState COLOR_TO_REJECT;

    // Action stuff
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    // bulk cache reading
    private List<LynxModule> allHubs;

    // adds ability to graph telemetry values
    private MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

    // loop time tracking
    private ElapsedTime elapsedtime;

    // rising and falling edge detection
    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        // init subsystems
        elapsedtime = new ElapsedTime();
        robotHardware.initialize(this);
        drive.initialize(this, robotHardware);
        verticalSlides.initialize(this, robotHardware, false);
        horizontalSlides.initialize(this, robotHardware, false);
        outtake.initialize(this, robotHardware);
        intake.initialize(this, robotHardware);
        hang.initialize(this, robotHardware);

        // bulk cache reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }
    }

    @Override
    public void init_loop() {
        // for rising edge detection
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if ((currentGamepad1.a && !previousGamepad1.a) || (currentGamepad2.a && !previousGamepad2.a)) {
            redAllianceBool = !redAllianceBool;
        }

        if ((currentGamepad1.b && !previousGamepad1.b) || (currentGamepad2.b && !previousGamepad2.b)) {
            sampleModeBool = !sampleModeBool;
        }
        telemetry.addData("red alliance? ", redAllianceBool);
        telemetry.addData("sample mode:  ", sampleModeBool);
    }

    @Override
    public void start() {
        COLOR_TO_REJECT = (redAllianceBool ? Intake.IntakeChamberState.BLUE : Intake.IntakeChamberState.RED);
        hang.disengagePTO();
        outtake.toStow();
        outtake.openClaw();
        intake.flipUp();
        verticalSlides.retract();
        horizontalSlides.retract();
    }

    @Override
    public void loop() {
        // clearing bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // for rising edge detection
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // for RR Action execution
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            if (action.run(packet)) { // actually running actions
                newActions.add(action); // if failed (run() returns true), try again
            }
        }
        runningActions = newActions;

        /// loops
        if (hangBool) {
            hang.operate(currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2);
            // failed wheely tilt, reset button
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                //undo wheely
                hang.setHangState(-1);
            }
            drive.operateHang();
        } else {
            drive.operateTeleOp();
            verticalSlides.operate();
        }

        horizontalSlides.operate();
        intake.operateColorChecking();

        /// updating booleans
        COLOR_TO_REJECT = (redAllianceBool ? Intake.IntakeChamberState.BLUE : Intake.IntakeChamberState.RED);
        drive.slowModeBool = !horizontalSlides.slidesRetracted || currentGamepad2.left_trigger > 0.1;
        drive.angleLockBool = (outtake.armPitch.armState == ExtendingOuttake.ArmPitch.STATE.GRABBING_CLIP) || (outtake.armPitch.armState == ExtendingOuttake.ArmPitch.STATE.SCORING_CLIP);
        if (currentGamepad1.left_trigger > 0.1 && !(previousGamepad1.left_trigger > 0.1)) { // avoids setting every loop since it probably takes time
            drive.setZeroPowerBrake(true);
        } else if (currentGamepad1.left_trigger <= 0.1 && !(previousGamepad1.left_trigger <= 0.1)) {
            drive.setZeroPowerBrake(false);
        }

        /// intake and transfer logic
        if (intake.intakeState == Intake.IntakeState.IDLE) {
            if (intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    // start intaking
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.dropDown()),
                            new InstantAction(() -> intake.intake())
                    ));
                }
            }
            else if (!sampleModeBool && intake.chamberState == Intake.IntakeChamberState.YELLOW) {
                runningActions.add(new InstantAction(() -> intake.reverse()));
            }
            else if (intake.chamberState != COLOR_TO_REJECT) {
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    // spit out (into human player zone or for teammate bucket bot)
                    runningActions.add(new InstantAction(() -> intake.reverse()));
                }
            } else if (intake.chamberState == COLOR_TO_REJECT) { // this case shouldn't ever be reached; wrong color should have been rejecting whilst intaking
                runningActions.add(new InstantAction(() -> intake.reverse()));
            }
        } else if (intake.intakeState == Intake.IntakeState.INTAKING) {
            if (intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                // keep trying to intake unless driver presses button
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    // flip up stop intaking
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.flipUp()),
                            new InstantAction(() -> intake.idle())
                    ));
                }
            } else if (!sampleModeBool && intake.chamberState == Intake.IntakeChamberState.YELLOW) {
                runningActions.add(new InstantAction(() -> intake.reverse()));
            } else if (intake.chamberState != COLOR_TO_REJECT) {
                // yay grabbed correct color sample, can stow now
//                gamepad1.rumble(500); /// rumble
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> gamepad1.rumble(400)), // maybe fixes massive delay
                        new InstantAction(() -> intake.flipUp()),
                        new InstantAction(() -> intake.idle())
                ));
            } else if (intake.chamberState == COLOR_TO_REJECT) {
                // reverse and go back to intaking
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> intake.flipUp()),
                        new SleepAction(0.3),
                        new ParallelAction( // this could be problematic
                                new InstantAction(() -> intake.reverse()),
                                new InstantAction(() -> intake.dropDown())
                        )
                ));
            }
        } else if (intake.intakeState == Intake.IntakeState.REVERSE) {
            // stop reversing when no more sample
            if (intake.wristFlippedUp && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                runningActions.add( new InstantAction(() -> intake.idle()) );
            } else if (!intake.wristFlippedUp && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> intake.dropDown()),
                        new InstantAction(() -> intake.intake())
                ));
            }
        }

        // backup force eject
        if (currentGamepad1.b && !previousGamepad1.b) {
            // run in reverse regardless of color sensor detection
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> intake.dropDown()),
                    new InstantAction(() -> intake.setIntake(-0.8)),
                    new SleepAction(0.6),
                    new InstantAction(() -> intake.idle())
            ));
        }

        /// outtake and scoring logic
        if (outtake.armPitch.armState == ExtendingOuttake.ArmPitch.STATE.SCORING_BUCKET) {
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                // deposit in bucket, then retract all
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> outtake.openClaw()),
                        new SleepAction(0.3),
                        new InstantAction(() -> outtake.toStow()),
                        new SleepAction(0.4),
                        new InstantAction(() -> verticalSlides.retract())
                ));
            }
        } else if (outtake.armPitch.armState == ExtendingOuttake.ArmPitch.STATE.SCORING_CLIP) {
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && !sampleModeBool) {
                // finish depositing clip and return to grab another
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> outtake.openClaw()),
                        new SleepAction(0.3),
                        new InstantAction(() -> outtake.retractExtender()),
                        new SleepAction(0.2),
                        new InstantAction(() -> outtake.toGrabClip()),
                        new InstantAction(() -> verticalSlides.retract())
                ));
            } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && sampleModeBool) {
                // finish depositing sample and return to stow
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> outtake.openClaw()),
                        new SleepAction(0.3),
                        new InstantAction(() -> outtake.retractExtender()),
                        new SleepAction(0.2),
                        new InstantAction(() -> outtake.toStow()),
                        new InstantAction(() -> verticalSlides.retract())
                ));
            }
        } else if (outtake.armPitch.armState == ExtendingOuttake.ArmPitch.STATE.GRABBING_CLIP) {
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                // grab clip and prep for scoring
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> outtake.closeClawTight()),
                        new SleepAction(0.3),
                        new InstantAction(() -> verticalSlides.raiseToPrepClip()),
                        new SleepAction(0.25),
                        new InstantAction(() -> outtake.armPitch.setArmScoreClip()), // decompose into two movements to add delay
                        new SleepAction(0.4),
                        new InstantAction(() -> outtake.armExtend.extendToScoreClip())
                ));
            }
        } else if (outtake.armPitch.armState == ExtendingOuttake.ArmPitch.STATE.STOW) {
            // full transfer sequence
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && horizontalSlides.slidesRetracted && intake.wristFlippedUp && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                if (highBucketBool) { // high bucket
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.setIntake(0.5)),
                            new InstantAction(() -> outtake.toTransfer()),
                            new SleepAction(0.2),
                            new InstantAction(() -> outtake.closeClawTight()),
                            new SleepAction(0.4),
                            new InstantAction(() -> outtake.toStow()),
                            new SleepAction(0.1),
                            new InstantAction(() -> intake.setIntake(0)),
                            new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                            new SleepAction(0.2),
                            new InstantAction(() -> outtake.toVert()),
                            new SleepAction(0.6),
                            new InstantAction(() -> outtake.toScoreBucket())
                    ));
                } else { // low bucket
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.setIntake(0.5)), // push sample all the way in, kinda jank, maybe not necessary
                            new InstantAction(() -> outtake.toTransfer()),
                            new SleepAction(0.2),
                            new InstantAction(() -> outtake.closeClawTight()),
                            new SleepAction(0.4),
                            new InstantAction(() -> outtake.toStow()),
                            new SleepAction(0.1),
                            new InstantAction(() -> intake.setIntake(0)),
                            new InstantAction(() -> verticalSlides.raiseToLowBucket()),
                            new InstantAction(() -> outtake.armPitch.setArmScoreBucket()),
                            new SleepAction(0.2),
                            new InstantAction(() -> outtake.armExtend.extendToScoreBucket())
                    ));
                }
            }
            else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                // flip over to grab clip
                drive.setTargetToCurrentHeading(); // change angle lock
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> verticalSlides.retract()),
                        new InstantAction(() -> outtake.openClaw()),
                        new InstantAction(() -> outtake.toGrabClip())
                ));
            }
        }

        // full reset button, in case macros mess up, or driver mixed up
        if (currentGamepad1.a && !previousGamepad1.a) {
            runningActions.add(new SequentialAction(
                    new ParallelAction(
                            new InstantAction(() -> intake.flipUp()),
                            new InstantAction(() -> intake.idle()),
                            new InstantAction(()-> outtake.toStow()),
                            new InstantAction(()-> outtake.openClaw())
                    ),
                    new SleepAction(0.7),
                    new ParallelAction(
                            new InstantAction(() -> verticalSlides.retract()),
                            new InstantAction(()-> horizontalSlides.retract())
                    )
            ));
        }

        // GAMEPAD 2 toggles, hang code above by loops
        if (currentGamepad2.a && !previousGamepad2.a) { // alliance color
            redAllianceBool = !redAllianceBool;
        }

        if (currentGamepad2.b && !previousGamepad2.b) { // sample spec mode
            sampleModeBool = !sampleModeBool;
        }

        // high low bucket toggle
        if (currentGamepad2.y && !previousGamepad2.y) { // high low bucket scoring
            highBucketBool = !highBucketBool;
        }

        // hang logic (this is where it gets messy :NOOOOO:)
        if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) {
            hangBool = !hangBool;
        }

        // telemetry
        telemetry.addData("Red alliance? ", redAllianceBool);
        telemetry.addData("Sample Mode: ", sampleModeBool);
        telemetry.addData("High Bucket Mode: ", highBucketBool);
        telemetry.addData("Hang Bool: ", hangBool);
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}

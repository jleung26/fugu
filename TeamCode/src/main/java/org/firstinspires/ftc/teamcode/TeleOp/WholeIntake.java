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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "B")
@Disabled
public class WholeIntake extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    Intake intake = new Intake();
    HorizontalSlides horiSlides = new HorizontalSlides();
    Mecanum drive = new Mecanum();

    private volatile boolean rejectBlue = true;

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
        horiSlides.initialize(this, robotHardware, true);
        intake.initialize(this, robotHardware);
        drive.initialize(this, robotHardware);

        // bulk cache reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }
    }

    @Override
    public void init_loop() {// for rising edge detection
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.a && !previousGamepad1.a) {
            rejectBlue = !rejectBlue;
        }
        telemetry.addData("reject Blue bool: ", rejectBlue);

    }

    @Override
    public void start() {
        intake.flipUp();
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

        // loops
        intake.operateColorChecking();
        horiSlides.operate();
        drive.operateTeleOp();

        // updating booleans
        drive.slowModeBool = !horiSlides.slidesRetracted;
        if (currentGamepad1.left_trigger > 0.1 && !(previousGamepad1.left_trigger > 0.1)) { // avoids setting every loop since it probably takes time
            drive.setZeroPowerBrake(true);
        } else if (currentGamepad1.left_trigger <= 0.1 && !(previousGamepad1.left_trigger <= 0.1)) {
            drive.setZeroPowerBrake(false);
        }
        Intake.IntakeChamberState COLOR_TO_REJECT = (rejectBlue ? Intake.IntakeChamberState.BLUE : Intake.IntakeChamberState.RED);

        // intake and transfer logic
        if (intake.intakeState == Intake.IntakeState.IDLE) {
            if (intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    // start intaking
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.dropDown()),
                            new InstantAction(() -> intake.intake())
                    ));
                }
            } else if (intake.chamberState != COLOR_TO_REJECT) {
                // full transfer sequence
                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && horiSlides.slidesRetracted) {
                    // transfer would be here
                }
                else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    // spit out (into human player zone or for teammate bucket bot)
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.reverse())
                    ));
                }
            } else if (intake.chamberState == COLOR_TO_REJECT) { // this case shouldn't ever be reached; wrong color should have been rejecting whilst intaking
                // reverse, then go back to idle
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> intake.reverse()),
                        new SleepAction(0.2),
                        new InstantAction(() -> intake.idle())
                ));
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
            } else if (intake.chamberState != COLOR_TO_REJECT) {
                // yay grabbed correct color sample, can stow now
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> intake.flipUp()),
                        new InstantAction(() -> intake.idle())
                ));
            } else if (intake.chamberState == COLOR_TO_REJECT && intake.prevChamberState == COLOR_TO_REJECT) {
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

        telemetry.addData("reject Blue bool: ", rejectBlue);
        telemetry.addData("color to reject: ", COLOR_TO_REJECT);
        telemetry.addData("color: ", intake.chamberState);
        telemetry.addData("intake state", intake.intakeState);
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}

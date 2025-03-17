package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
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
        drive.operateSimple();

        Intake.IntakeChamberState COLOR_TO_REJECT = (rejectBlue ? Intake.IntakeChamberState.BLUE : Intake.IntakeChamberState.RED);

        if (intake.intakeState == Intake.IntakeState.NEUTRAL) {
            if (intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                // button press
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.dropDown()),
                            new InstantAction(() -> intake.intake())
                    ));
                }
            } else if (intake.chamberState != COLOR_TO_REJECT) {
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.reverse()),
                            new SleepAction(0.3),
                            new InstantAction(() -> intake.neutral())
                    ));
                }
                // eventually, for spec, it will be spit out
                // for sample mode, it will be transfer
            } else if (intake.chamberState == COLOR_TO_REJECT) {

                runningActions.add(new SequentialAction(
                         new InstantAction(() -> intake.reverse()),
                         new SleepAction(0.2),
                         new InstantAction(() -> intake.neutral())
                 ));
            }
        } else if (intake.intakeState == Intake.IntakeState.INTAKING) {
            if (intake.chamberState == Intake.IntakeChamberState.EMPTY) {
                // keep going unless below button pressed
                if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.flipUp()),
                            new InstantAction(() -> intake.neutral())
                    ));
                }
            } else if (intake.chamberState != COLOR_TO_REJECT) {
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> intake.flipUp()),
                        new InstantAction(() -> intake.neutral())
                ));
            } else if (intake.chamberState == COLOR_TO_REJECT && intake.prevChamberState == COLOR_TO_REJECT) {
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> intake.flipUp()),
                        new InstantAction(() -> intake.reverse()),
                        new SleepAction(0.3),
                        new InstantAction(() -> intake.dropDown()),
                        new InstantAction(() -> intake.intake())
                ));
            }
        } else if (intake.intakeState == Intake.IntakeState.REVERSE) {
            // nothing
        }

        telemetry.addData("reject Blue bool: ", rejectBlue);
        telemetry.addData("color to reject: ", COLOR_TO_REJECT);
        telemetry.addData("color: ", intake.chamberState);
        telemetry.addData("intake state", intake.intakeState);
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}

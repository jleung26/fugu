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

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Subsystem.SimpleHori;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class WholeIntake extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    Intake intake = new Intake();
    SimpleHori horiSlides = new SimpleHori();
    Mecanum drive = new Mecanum();
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

        intake.flipUp();
        intake.neutral();
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
            action.preview(packet.fieldOverlay()); // maybe unnecessary, will test eventually
            if (action.run(packet)) { // actually running actions
                newActions.add(action); // if failed (run() returns true), try again
            }
        }
        runningActions = newActions;
        dash.sendTelemetryPacket(packet);

        // loops
        intake.operateColorChecking(false);
        horiSlides.operateTest(packet);
        drive.operateSimple();


         if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && intake.chamberState == Intake.IntakeChamberState.EMPTY) {
             intake.dropDown();
             intake.intake();
         } else if (currentGamepad1.a && !previousGamepad1.a && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
             intake.dropDown();
             intake.reverse();
             runningActions.add(new SequentialAction(
                     new SleepAction(1),
                     new InstantAction(() -> intake.flipUp()),
                     new InstantAction(() -> intake.neutral())
             ));
         } else if (intake.chamberState != Intake.IntakeChamberState.EMPTY && intake.intakeState != Intake.IntakeState.REVERSE) {
             intake.flipUp();
             intake.neutral();
         }


        dashboardTelemetry.update();

        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}

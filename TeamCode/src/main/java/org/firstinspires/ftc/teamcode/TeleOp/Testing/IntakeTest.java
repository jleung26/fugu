package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.List;

@TeleOp(group = "Testing")
public class IntakeTest extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    Intake intake = new Intake();
    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    private List<LynxModule> allHubs;

    private ElapsedTime elapsedtime;

    boolean testingManualMode = true;
    boolean rejectBlue = true;

    @Override
    public void init() {
        robotHardware.initialize(this);
        intake.initialize(this, robotHardware);
        elapsedtime = new ElapsedTime();
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }
    }

    @Override
    public void start() {
        elapsedtime.reset();
    }

    @Override
    public void loop() {
        // bulk caching so we don't have egregious loop times from all the i2c calls I'm making
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Final testing
        // 1. press right bumper to toggle into automated mode
        //    (automated mode should auto reject when wrong and auto stop when correct)
        // 2. toggle X to change which alliance color to reject
        // 3. suck up the whole submersible

        // boolean toggle
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            testingManualMode = !testingManualMode;
        }

        // another boolean toggle
        if (currentGamepad1.x && !previousGamepad1.x) {
            rejectBlue = !rejectBlue;
        }

        if (testingManualMode) {
            intake.operateTesting();
        }else {
            intake.operateColorChecking(rejectBlue);
        }

        telemetry.addData("manual mode: ", testingManualMode);
        telemetry.addData("reject Blue: ", rejectBlue);
        telemetry.addData("loop time", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}

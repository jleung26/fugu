package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.List;

@TeleOp
public class MecanumTest extends OpMode {
    // simple driving and PD turning
    private Mecanum drive = new Mecanum();
    RobotHardware robotHardware;
    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;

    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        robotHardware.initialize(this);
        drive.initialize(this, robotHardware);

        // loop time stuff
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        // bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // for rising edge detection
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // drive robot-centric default
        // left trigger - slowmode
        // right trigger - auto rotate to target
        // B- reset
        // A - set target
        drive.operateTesting();

        // gyro reset
        if (gamepad1.b) {drive.pinpoint.softResetYaw();}

        // sets target
        if (gamepad1.a) {drive.targetAngle = drive.pinpoint.relativeNormalizedHeading;}

        // loop time measuring
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}
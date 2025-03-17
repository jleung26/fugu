package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Util.PinpointManager;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp(group = "Testing")
public class PinpointTest extends OpMode {
    RobotHardware robotHardware  = new RobotHardware();
    PinpointManager pinpoint = new PinpointManager();

    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        robotHardware.initialize(this);
        pinpoint.initialize(this, robotHardware);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.a && !previousGamepad1.a) {
            pinpoint.softResetYaw();
        }

        pinpoint.operateTesting();
    }
}

package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystem.Hang;
import org.firstinspires.ftc.teamcode.Subsystem.Mecanum;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp(group = "Testing")
public class ManualHangTest extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    private Hang hang = new Hang();
    private Mecanum mecanum = new Mecanum();

    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        robotHardware.initialize(this);
        mecanum.initialize(this, robotHardware);
        hang.initialize(this, robotHardware);
    }

    @Override
    public void start() {
        hang.setHangState(0);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        hang.operate(currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2);
        // failed wheely tilt, reset button
        if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
            //undo wheely
            hang.setHangState(-1);
        }
        mecanum.operateHang();
    }
}

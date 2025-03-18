package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Hang;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp(group = "Testing")
public class ManualHangTest extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    private Hang hang = new Hang();

    @Override
    public void init() {
        robotHardware.initialize(this);
        hang.initialize(this, robotHardware);
    }

    @Override
    public void loop() {
        hang.operateSetPos();
    }
}

package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.ExtendingOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp(group = "Testing")
public class OuttakeTest extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    private ExtendingOuttake outtake = new ExtendingOuttake();

    @Override
    public void init() {
        robotHardware.initialize(this);
        outtake.initialize(this, robotHardware);
    }

    @Override
    public void loop() {
        outtake.operateTuning();
    }
}

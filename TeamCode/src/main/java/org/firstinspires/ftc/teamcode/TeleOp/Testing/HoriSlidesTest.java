package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp
public class HoriSlidesTest extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    private FtcDashboard dash = FtcDashboard.getInstance();
    private MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
    private HorizontalSlides horizontalSlides = new HorizontalSlides();

    @Override
    public void init() {
        robotHardware.initialize(this);
        horizontalSlides.initialize(this, robotHardware, true);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        horizontalSlides.operateTuning(packet);

        dash.sendTelemetryPacket(packet);

        dashboardTelemetry.update();
    }
}

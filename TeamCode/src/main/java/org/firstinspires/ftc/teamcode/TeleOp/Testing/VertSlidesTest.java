package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp
public class VertSlidesTest extends OpMode {
    RobotHardware robotHardware;
    private VerticalSlides verticalSlides = new VerticalSlides();
    private ElapsedTime elapsedtime;


    @Override
    public void init() {
        robotHardware.initialize(this);
        verticalSlides.initialize(this, robotHardware, true);

        // loop time stuff
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }

    @Override
    public void loop() {
        verticalSlides.operateTuning();

        // loop time measuring
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}

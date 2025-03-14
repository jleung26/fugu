package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp
public class HoriSlidesTest extends OpMode {
    RobotHardware robotHardware;
    private HorizontalSlides horizontalSlides = new HorizontalSlides();

    @Override
    public void init() {
        robotHardware.initialize(this);
        horizontalSlides.initialize(this, robotHardware, true);
    }

    @Override
    public void loop() {
        horizontalSlides.operateTuning();
    }
}

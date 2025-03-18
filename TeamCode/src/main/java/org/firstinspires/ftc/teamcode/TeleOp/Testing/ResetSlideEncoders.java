package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@TeleOp(group = "A")
public class ResetSlideEncoders extends OpMode {
    RobotHardware robotHardware = new RobotHardware();
    VerticalSlides verticalSlides = new VerticalSlides();
    HorizontalSlides horizontalSlides = new HorizontalSlides();

    @Override
    public void init() {
        robotHardware.initialize(this);
        verticalSlides.initialize(this, robotHardware, true);
        horizontalSlides.initialize(this, robotHardware, true);
    }

    @Override
    public void start() {
//        requestOpModeStop(); // hopefully this doesn't break anything, otherwise, can just remove
    }

    @Override
    public void loop() {

    }
}

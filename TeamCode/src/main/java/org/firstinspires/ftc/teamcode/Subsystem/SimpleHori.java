package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Util.RobotHardware;

@Config
public class SimpleHori {
    OpMode opmode;
    public DcMotorEx horiMotor;
    private PIDController controller;

    public static double Kp = 0.0045;
    public static double Ki = 0;
    public static double Kd = 0.00001;

    public static double extendedPos = 900;
    public static double MAPPING_EXPONENT = 1;

    public static double target = 0;
    private volatile double output = 0;

    public SimpleHori() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware, boolean resetEncoders) {
        this.opmode = opmode;
        this.horiMotor = robotHardware.horiMotor;

        controller = new PIDController(Kp, Ki, Kd);

//        if (resetEncoders) {
//            horiMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
    }

    public void operateTest(TelemetryPacket packet) {
        controller.setPID(Kp, Ki, Kd);
        int currentPos = horiMotor.getCurrentPosition();

//        if (opmode.gamepad1.a) {
//            target = 800;
//        } else if (opmode.gamepad1.b) {
//            target = 0;
//        }

        target = mapTriggerToTarget(opmode.gamepad1.right_trigger);
        output = controller.calculate(currentPos, target);
        output = (target < currentPos ? -1 : 1) * Math.sqrt(Math.abs(output));
        horiMotor.setPower(output);

//        if (opmode.gamepad1.left_trigger > 0.1) {
//            output = controller.calculate(currentPos, target);
//            horiMotor.setPower(output);
//        } else {
//            horiMotor.setPower(-opmode.gamepad1.left_stick_y);
//        }

        opmode.telemetry.addData("target", target);
        opmode.telemetry.addData("output power", output);
        opmode.telemetry.addData("current pos: ", currentPos);
//        opmode.telemetry.addData("left stick y", -opmode.gamepad1.left_stick_y);

        packet.put("target", target);
        packet.put("current pos ", currentPos);
    }

    public int mapTriggerToTarget(double input) {
        return (int) Math.round(Math.pow(input, MAPPING_EXPONENT) * extendedPos);
        // paste this into desmos to see graph: x^{0.4}\ \left\{0\le x\le1\right\}
        // making the mapping exponent smaller makes the graph steeper
    }
}

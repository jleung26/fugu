package org.firstinspires.ftc.teamcode.Util;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    private OpMode opmode;
    public DcMotorEx Fl, Fr, Bl, Br, leftVertSlideMotor, rightVertSlideMotor, horiSlideMotor;
    public CRServo leftHangServo, rightHangServo;

    public Servo clipServo, intakeWrist;

    public GoBildaPinpointDriver odo;

    public void initialize(OpMode opmode) {
        /*
            CONFIG SHEET:

            Br - CHUB 3
            Fr - CHUB 2
            Bl - EHUB 3
            Fl - EHUB 2

            rightHang - CHUB SERVO 0
            leftHang - EHUB SERVO 5

            pinpoint - CHUB i2c 1

            hori - CHUB 0
            right vert - CHUB 1
            left vert - EHUB 1

         */
//        this.opmode = opmode;

        Fl = opmode.hardwareMap.get(DcMotorEx.class, "leftFront");
        Fr = opmode.hardwareMap.get(DcMotorEx.class, "rightFront");
        Bl = opmode.hardwareMap.get(DcMotorEx.class, "leftRear");
        Br = opmode.hardwareMap.get(DcMotorEx.class, "rightRear");

        //TODO: configure computer after
        odo = opmode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        leftVertSlideMotor = opmode.hardwareMap.get(DcMotorEx.class, "leftVert");
        rightVertSlideMotor = opmode.hardwareMap.get(DcMotorEx.class, "rightVert");

        horiSlideMotor = opmode.hardwareMap.get(DcMotorEx.class, "horiSlide");

        leftHangServo = opmode.hardwareMap.get(CRServo.class, "leftHang");
        rightHangServo = opmode.hardwareMap.get(CRServo.class, "rightHang");

        intakeWrist = opmode.hardwareMap.get(Servo.class, "intakeWrist");
        clipServo = opmode.hardwareMap.get(Servo.class, "clipper");

        Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        Fl.setDirection(DcMotorSimple.Direction.REVERSE);
        Bl.setDirection(DcMotorSimple.Direction.REVERSE);

        leftVertSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightVertSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        horiSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftVertSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVertSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horiSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftVertSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftHangServo.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setOffsets(-131.749, 126);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }
}

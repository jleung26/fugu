package org.firstinspires.ftc.teamcode.Util;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    public DcMotorEx Fl, Fr, Bl, Br, leftVertMotor, rightVertMotor, horiMotor, intakeMotor;

    public Servo intakeWrist, leftWheelyServo, rightWheelyServo, leftPtoServo, rightPtoServo, armAngleServo, armPitchServo, armClawServo;

    public GoBildaPinpointDriver odo;

    public void initialize(OpMode opmode) {
        /*
            CONFIG SHEET:

            pinpoint - CHUB i2c 1
            color sensor - CHUB i2c ____

            MOTOR

            Br - EHUB ___
            Fr - EHUB ___
            Bl - EHUB __
            Fl - EHUB __
            hori - CHUB ___ (w/ encoder)
            right vert - CHUB ___ (w/ encoder)
            left vert - CHUB ____
            intake - CHUB ____

            SERVO

            intake wrist - CHUB ___


         */
        odo = opmode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        Fl = opmode.hardwareMap.get(DcMotorEx.class, "leftFront");
        Fr = opmode.hardwareMap.get(DcMotorEx.class, "rightFront");
        Bl = opmode.hardwareMap.get(DcMotorEx.class, "leftRear");
        Br = opmode.hardwareMap.get(DcMotorEx.class, "rightRear");

        leftVertMotor = opmode.hardwareMap.get(DcMotorEx.class, "leftVert");
        rightVertMotor = opmode.hardwareMap.get(DcMotorEx.class, "rightVert");
        horiMotor = opmode.hardwareMap.get(DcMotorEx.class, "horiSlide");
        intakeMotor = opmode.hardwareMap.get(DcMotorEx.class, "intake");


        intakeWrist = opmode.hardwareMap.get(Servo.class, "intakeWrist");

        leftWheelyServo = opmode.hardwareMap.get(Servo.class, "");
        rightWheelyServo = opmode.hardwareMap.get(Servo.class, "");
        leftPtoServo = opmode.hardwareMap.get(Servo.class, "");
        rightPtoServo = opmode.hardwareMap.get(Servo.class, "");

        armAngleServo = opmode.hardwareMap.get(Servo.class, "");
        armPitchServo = opmode.hardwareMap.get(Servo.class, "");
        armClawServo = opmode.hardwareMap.get(Servo.class, "");

        Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        Fl.setDirection(DcMotorSimple.Direction.REVERSE);
        Bl.setDirection(DcMotorSimple.Direction.REVERSE);

        leftVertMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightVertMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        horiMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftVertMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVertMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horiMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO
        leftVertMotor.setDirection(DcMotorSimple.Direction.FORWARD); // not too sure which one yet, depends on spooling
        rightVertMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        horiMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // TODO: offsets, directions (not actually necessary)
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }
}

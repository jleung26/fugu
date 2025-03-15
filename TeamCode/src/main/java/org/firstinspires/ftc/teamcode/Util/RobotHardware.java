package org.firstinspires.ftc.teamcode.Util;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    public DcMotorEx Fl, Fr, Bl, Br, leftVertMotor, rightVertMotor, horiMotor, intakeMotor;

    public Servo leftIntakeWristServo, rightIntakeWristServo, leftWheelyServo, rightWheelyServo, leftPtoServo, rightPtoServo, armAngleServo, armPitchServo, armClawServo;

    public GoBildaPinpointDriver odo;

    public RevColorSensorV3 intakeColorSensor;

    public void initialize(OpMode opmode) {
        /*
            CONFIG:

            pinpoint - CHUB i2c 2
            color sensor - CHUB i2c 1

            MOTOR

            Br - EHUB 3
            Bl - EHUB 2
            Fr - EHUB 1
            Fl - EHUB 0
            intake - CHUB 0
            hori - CHUB 1 (w/ encoder)
            left vert - CHUB 2 (w/ encoder)
            right vert - CHUB 3

            SERVO

            intake right wrist - EHUB 0
            intake left wrist - EHUB 2

            left PTO - CHUB 4
            right PTO - CHUB 5

            arm arm - EHUB 1
            arm wrist - EHUB 3
            arm claw - EHUB 5

            left wheely - EHUB 2
            right wheely - EHUB 0

         */
        odo = opmode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        intakeColorSensor = opmode.hardwareMap.get(RevColorSensorV3.class, "color");

        Fl = opmode.hardwareMap.get(DcMotorEx.class, "leftFront");
        Fr = opmode.hardwareMap.get(DcMotorEx.class, "rightFront");
        Bl = opmode.hardwareMap.get(DcMotorEx.class, "leftRear");
        Br = opmode.hardwareMap.get(DcMotorEx.class, "rightRear");

        leftVertMotor = opmode.hardwareMap.get(DcMotorEx.class, "leftVert");
        rightVertMotor = opmode.hardwareMap.get(DcMotorEx.class, "rightVert");
        horiMotor = opmode.hardwareMap.get(DcMotorEx.class, "horiSlide");
        intakeMotor = opmode.hardwareMap.get(DcMotorEx.class, "intake");

        leftIntakeWristServo = opmode.hardwareMap.get(Servo.class, "leftIntakeWrist");
        rightIntakeWristServo = opmode.hardwareMap.get(Servo.class, "rightIntakeWrist");

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

        leftVertMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // has encoder
        rightVertMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horiMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // has encoder

        leftVertMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightVertMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        horiMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftIntakeWristServo.setDirection(Servo.Direction.REVERSE);
        rightPtoServo.setDirection(Servo.Direction.REVERSE);
        leftWheelyServo.setDirection(Servo.Direction.REVERSE);

        // TODO: offsets, directions (not actually necessary)
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }
}

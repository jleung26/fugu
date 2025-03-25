package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.pedropathing.localization.GoBildaPinpointDriver;


import java.util.Locale;

@Config
public class PinpointManager { // pinpoint for use during teleop, pedro has its own implementation
    GoBildaPinpointDriver odo;
    OpMode opmode;
    Telemetry telemetry;

    double oldTime;

    // all of the following are in degrees
    public volatile double absoluteHeading, normalizedHeading, relativeNormalizedHeading, offset;

    public PinpointManager() {}

    public void initialize(OpMode opmode, RobotHardware robotHardware) {
        this.opmode = opmode;
        this.telemetry = opmode.telemetry;

        odo = robotHardware.odo;

        odo.resetPosAndIMU();

        offset = 0;

        opmode.telemetry.addData("Status", "Initialized");
        opmode.telemetry.update();
    }

    public void operateTesting() {
        odo.update();

        absoluteHeading = Math.toDegrees(odo.getHeading());
        normalizedHeading = normalize(absoluteHeading);
        relativeNormalizedHeading = normalize(normalizedHeading - offset);

        telemetry.addData("UNnormalized absolute heading (degrees)", absoluteHeading);
        telemetry.addData("normalized absolute heading (degrees)", normalizedHeading);
        telemetry.addData("normalized relative heading (degrees)", relativeNormalizedHeading);
        telemetry.addData("offset (degrees)", offset);

        // loop times
        double newTime = opmode.getRuntime();
        double frequency = 1/(newTime-oldTime);
        oldTime = newTime;
        telemetry.addData("Pinpoint Frequency (Hz)", odo.getFrequency()); // pinpoint refresh rate
        telemetry.addData("REV Hub Frequency (Hz): ", frequency); // control hub refresh rate

        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f mm, Y: %.3f mm, Heading: %.3f°}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Status", odo.getDeviceStatus());
        telemetry.update();
    }

    public void operateSimple() {
        odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);

        normalizedHeading = normalize(Math.toDegrees(odo.getHeading()));
        relativeNormalizedHeading = normalize(normalizedHeading - offset);
    }

    public void softResetYaw() {
        offset += normalizedHeading;
    }

    public double normalize(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}
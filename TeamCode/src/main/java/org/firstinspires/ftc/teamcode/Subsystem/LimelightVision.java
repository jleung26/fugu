package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;
import org.opencv.objdetect.DetectorParameters;

import java.util.List;

public class LimelightVision {
    private OpMode opmode;
    private RobotHardware rHardware;
    private Limelight3A limelight;

    private LLResult result;

    public void initialize(OpMode opmode, RobotHardware RH) {
        this.opmode = opmode;
        this.limelight = RH.limelight;

        limelight.setPollRateHz(100);
        opmode.telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void operate() {
        result = limelight.getLatestResult();

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        for (LLResultTypes.DetectorResult detection: detections) {
            String className = detection.getClassName();
            double x = detection.getTargetXDegrees();
            double y = detection.getTargetYDegrees();
            opmode.telemetry.addData(className, "at (" + x + "," + y + ") degrees");
        }

        long staleness = result.getStaleness();
        if (staleness < 100) { // Less than 100 milliseconds old
            opmode.telemetry.addData("Data", "Good");
        } else {
            opmode.telemetry.addData("Data", "Old (" + staleness + " ms)");
        }
    }
}

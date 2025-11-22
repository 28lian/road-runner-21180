package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;

import java.util.List;


@Config
public final class Colored {
    public final Limelight3A limelight;

    public Colored(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);  // poll 100x per second
        limelight.start();
        limelight.pipelineSwitch(1);
    }
    public void setOrientation(double robotYaw) {
        limelight.updateRobotOrientation(robotYaw);
    }
    public Pose3D returnPositionbeta() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getBotpose();
        } else {
            return null;
        }
    }
    public double[] returnPosition() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int fiducialId = fiducial.getFiducialId();
                if (fiducialId == 20 || fiducialId == 24)
                {
                    double x = fiducial.getTargetXDegrees();
                    double y = fiducial.getTargetYDegrees();
                    double z = fiducial.getTargetArea();
                    return new double[]{x, y, z};
                }
                else
                {
                    return new double[] {0, 0, 0};
                }
            }
        }
        else
        {
            return new double[] {0, 0, 0};
        }

//        double[] pythonOutputs = result.getPythonOutput();
//        if (pythonOutputs == null || pythonOutputs.length < 5)
//        {
//            return new double[0];
//        }
//
//        // Each detection is 5 values long: [colorCode, x, y, w, h]
//        for (int i = 0; i < pythonOutputs.length; i += 5) {
//            double colorCode = pythonOutputs[i];
////            Logging.log("color code[%d] =  %f", i, colorCode);
//            if (colorCode == 20 || colorCode == 24) {
//                Logging.log("color code found = %f", colorCode);
//                double x = pythonOutputs[i + 1];
//                double y = pythonOutputs[i + 2];
//                return new double[]{x, y};
//            }
//        }
        return new double[0]; // no valid detection
    }

    public double returnId(boolean goal) {
        LLResult result = limelight.getLatestResult();
        double detectedID = 0;
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int fiducialId = fiducial.getFiducialId();
                if (goal) {
                    if (fiducialId == 20 || fiducialId == 24) {
                        detectedID = fiducialId;
                    }
                } else {
                    if (fiducialId == 21 || fiducialId == 22 || fiducialId == 23) {
                        detectedID = fiducialId;
                    }
                }
            }
        }
//        double[] pythonOutputs = result.getPythonOutput();
//        if (pythonOutputs == null || pythonOutputs.length < 5)
//        {
//            return 0;
//        }

//        // Each detection is 5 values long: [colorCode, x, y, w, h]
//        for (int i = 0; i < pythonOutputs.length; i += 5) {
//            double colorCode = pythonOutputs[i];
//            Logging.log("color code[%d] =  %f", i, colorCode);
//            // x, y, w, h not needed right now
//            if (colorCode == 21 || colorCode == 22 || colorCode == 23) {
//                return colorCode;
//            }
//        }

        return detectedID; // no valid detection
    }
    public String GetDiagnostics() {
        LLStatus diags = new LLStatus();
        return diags.toString();
    }

    public int returnPipeline() {
        LLResult result = limelight.getLatestResult();
        return result.getPipelineIndex();
    }
}

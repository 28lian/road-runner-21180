package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "LimelightDistance", group = "Concept")
@Disabled
public class LimelightDistance extends LinearOpMode {
    private Limelight3A limelight;

    public double[] returnPosition() {
        // Get latest result from Limelight
        LLResult result = limelight.getLatestResult();
        if (result == null) return null;

        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs == null || pythonOutputs.length < 5) return null;
        // if not iterable as shown below, return null
        if (pythonOutputs.length % 5 != 0) {
             return null;
        }
        // Each detection is 5 values long: [goal code, x, y, w, h]
        for (int i = 0; i < pythonOutputs.length; i += 5) {
            double goalCode = pythonOutputs[i];
            double x = pythonOutputs[i + 1];
            double y = pythonOutputs[i + 2];
            double w = pythonOutputs[i + 3];
            double h = pythonOutputs[i + 4];

            if (goalCode == 20 || goalCode == 24) {
                // Return array of all 5 values
                return new double[] {goalCode, x, y, w, h};
            }
        }

        // No valid detection found
        return null;
    }

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);  // poll 100x per second
        limelight.start();
        limelight.pipelineSwitch(2);
        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double[] detection = returnPosition();
                if (detection != null) {
                    telemetry.addData("Goal Code", detection[0]);
                    telemetry.addData("X", detection[1]);
                    telemetry.addData("Y", detection[2]);
                    telemetry.addData("Width", detection[3]);
                    telemetry.addData("Height", detection[4]);
                } else {
                    telemetry.addLine("No valid detection");
                }
                telemetry.update();
            }
        }
    }
}

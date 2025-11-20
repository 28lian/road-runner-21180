package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "testprog", group = "Concept")
@Disabled
public class testprogram extends LinearOpMode {
    // get the software-hardware links ready
    private intakeUnit2026 motors;

    @Override
    public void runOpMode() {
        // connect the hardware map to color discrimination system and prepare launcher, intake, and trigger
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        waitForStart();
        if (opModeIsActive()) {
            run_auto();
        }
    }

    private void run_auto() {
        motors.triggerClose();

        shootArtifacts();
    }

    // function to shoot 3 artifacts
    private void shootArtifacts() {
        int waitTimeForTriggerClose = 300;
        int waitTimeForTriggerOpen = 950;
        Logging.log("start shooting.");
        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < 0.1) {
            Logging.log("start launcher motor since it is stopped.");
            motors.startLauncherFar();
            sleep(waitTimeForTriggerOpen + 500); // waiting time for launcher motor ramp up
        }

        motors.triggerOpen(); // shoot first
        sleep(waitTimeForTriggerClose);
        motors.triggerClose(); //close trigger to wait launcher motor speed up after first launching

        motors.startIntake(); // start intake motor to move 3rd artifacts into launcher
        sleep(waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot second
        sleep(waitTimeForTriggerClose);

        motors.triggerClose();
        sleep(waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot third
        sleep(waitTimeForTriggerClose + 150);

        motors.triggerClose();
        motors.stopLauncher();
    }
}
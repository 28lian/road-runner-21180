package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Far Blue shoot 2026", group = "Concept")
public class AutoFarBlue2026 extends LinearOpMode {
    private MecanumDrive drive;
    private intakeUnit2026 motors;

    public int leftOrRight = 1;

    Vector2d endPose;
    Pose2d launchPose;

    public void setSide() {
        leftOrRight = 1;
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        setSide();
        Params.leftOrRight = leftOrRight;
        // Starting pose
        Pose2d startPose = new Pose2d(
                (-6 * Params.HALF_MAT + Params.CHASSIS_HALF_LENGTH),
                (leftOrRight * Params.CHASSIS_HALF_WIDTH),
                Math.toRadians(-180)
        );

        //drive here to launch
        launchPose = new Pose2d(- 4.5 * Params.HALF_MAT, leftOrRight * Params.HALF_MAT, Math.toRadians(motors.launchDegreeFar * leftOrRight));

        //drive here after launching
        endPose = new Vector2d(- 5 * Params.HALF_MAT, leftOrRight * 3 * Params.HALF_MAT);

        drive = new MecanumDrive(hardwareMap, startPose);
        motors.triggerClose();

        waitForStart();
        if (opModeIsActive()) {
            run_auto();
            Params.currentPose = drive.localizer.getPose(); // save current position
        }
    }

    private void run_auto() {

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(launchPose.position, launchPose.heading)
                        .build()
        );

        sleep(1); // waiting near alliance shooting.
        shootArtifacts(true);

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(endPose, Math.toRadians(180))
                        .build()
        );

    }

    public void shootArtifacts(boolean farLaunch) {
        int waitTimeForTriggerClose = 300;
        int waitTimeForTriggerOpen = 1000; // TODO: checking if it is ok for far shooting
        int rampUpTime = 1000;

        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < 0.1) {
            if (farLaunch) {
                motors.startLauncherFar(); // far power
            }
            else {
                motors.startLauncherFar(); // near
            }
            sleep(rampUpTime); // waiting time for launcher motor ramp up
        }
        // launcher is started but with near launching power
        else if ((motors.getLaunchVelocity() < motors.launchSpeedFar * 0.9) && farLaunch) {
            motors.startLauncherFar();
        }
        // launcher is started but with higher power
        else if ((motors.getLaunchVelocity() > motors.launchSpeedNear * 1.1) && !farLaunch)
        {
            motors.startLauncherFar();
        }
        sleep(rampUpTime);

        motors.triggerOpen(); // shoot first
        sleep(waitTimeForTriggerClose);
        motors.triggerClose(); //close trigger to wait launcher motor speed up after first launching

        // reset launcher power for the left artifacts
        if (farLaunch)
        {
            motors.startLauncherFar();
        }
        else
        {
            motors.startLauncher();
        }

        motors.startIntake(); // start intake motor to move 3rd artifacts into launcher
        sleep(waitTimeForTriggerOpen);// waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot second
        sleep(waitTimeForTriggerClose);

        motors.triggerClose();
        motors.setLauncherVelocity(motors.launchDegreeFar - 4);
        sleep(waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen();  // shoot third
        sleep(waitTimeForTriggerClose);

        motors.triggerClose();
        motors.stopIntake();
        motors.stopLauncher();
    }
}
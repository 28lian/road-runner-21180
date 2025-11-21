package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Far Blue shoot pickup", group = "Concept")
public class AutoFarBluePick26 extends LinearOpMode {
    private MecanumDrive drive;
    private intakeUnit2026 motors;
    private final ElapsedTime runtime = new ElapsedTime();

    public int leftOrRight = 1;

    public int pickRowNum = 1;

    Vector2d endPose;
    Pose2d launchPose;

    public void setSide() {
        leftOrRight = 1;
    }

    public void setPickupRowNum() {
        pickRowNum = 1;
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        // blue = 1; red = -1;
        setSide();

        //pickup row number = 0, 1, 2, or 3;
        setPickupRowNum();

        Params.leftOrRight = leftOrRight;
        // Starting pose
        Pose2d startPose = new Pose2d(
                (-6 * Params.HALF_MAT + Params.CHASSIS_HALF_LENGTH),
                (leftOrRight * Params.CHASSIS_HALF_WIDTH),
                Math.toRadians(-180)
        );

        Params.currentPose = startPose; // save the Position

        //drive here to launch
        launchPose = new Pose2d(- 4.5 * Params.HALF_MAT, leftOrRight * Params.HALF_MAT, Math.toRadians(motors.launchDegreeFar * leftOrRight));

        //drive here after launching
        endPose = new Vector2d(- 5 * Params.HALF_MAT, leftOrRight * 3 * Params.HALF_MAT);

        drive = new MecanumDrive(hardwareMap, startPose);
        motors.triggerClose();

        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
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
        shootArtifactsFar();

        // the following is a velocity constraint for moving to pick up artifacts
        VelConstraint pickupSpeed = (robotPose, _path, _disp) -> 9.0;

        for (int pickupIndex = 0; pickupIndex < pickRowNum; pickupIndex++) {
            Vector2d pickupPos;
            Vector2d pickupEndPos;
            // 23 is the closest row to start position, then 22, then 21, so new if statement below will optimize pathing
            pickupPos = new Vector2d(
                    (-3 * 2 + 3) * Params.HALF_MAT - (3 - 1) / 2.0, // add additional inch according to testing.
                    leftOrRight * (2.2 * Params.HALF_MAT + Params.CHASSIS_HALF_LENGTH / 2)
            );

            // fixed polarity below (there was a double negative sign before)
            pickupEndPos = new Vector2d(pickupPos.x, pickupPos.y + 1.4 * Params.HALF_MAT * Math.signum(pickupPos.y));

            // action for picking up artifacts
            Action actMoveToPickup = drive.actionBuilder(drive.localizer.getPose())
                    // add 4 degree more for row1 according to test results
                    .strafeToLinearHeading(pickupPos, Math.toRadians(90.0 * leftOrRight))
                    .build();
            Actions.runBlocking(actMoveToPickup); // ready for pickup artifacts

            // close launching trigger before pickup artifacts
            motors.stopLauncher();

            // starting intake motor
            motors.startIntake();
            Action actIntake = drive.actionBuilder(drive.localizer.getPose())
                    .strafeToConstantHeading(pickupEndPos, pickupSpeed) // picking up artifacts
                    .build();
            Actions.runBlocking(actIntake); // complete pickup artifacts

            Action actMoveToLaunch = drive.actionBuilder(drive.localizer.getPose())
                    .afterTime(0.3, new startLauncherAction()) // start launcher motor
                    .strafeToLinearHeading(launchPose.position, launchPose.heading)
                    .build();
            Actions.runBlocking(actMoveToLaunch);

            // shoot picked up artifacts
            shootArtifactsFar();
        }

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(endPose, Math.toRadians(180))
                        .build()
        );

    }

    private void shootArtifactsFar() {
        int waitTimeForTriggerClose = 3000;
        int waitTimeForTriggerOpen = 700;
        int rampUpTime = 1000;
        double targetV = motors.launchSpeedFar;

        Logging.log("start shooting.");
        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < 0.1) {
            Logging.log("start launcher motor since it is stopped.");
            motors.startLauncherFar();
        }
        reachTargetVelocity(targetV, rampUpTime); // waiting time for launcher motor ramp up

        motors.triggerOpen(); // shoot first
        Logging.log("launcher velocity for #1 one: %.1f.", motors.getLaunchVelocity());
        velocityRampDown(targetV, waitTimeForTriggerClose);
        Logging.log("starting close trigger.");
        motors.triggerClose(); //close trigger to wait launcher motor speed up after first launching


        // starting shoot 2nd one
        targetV = targetV - 5;
        motors.startIntake(); // start intake motor to move 3rd artifacts into launcher
        reachTargetVelocity(targetV, waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot second
        Logging.log("launcher velocity for #2 one: %.2f.", motors.getLaunchVelocity());

        velocityRampDown(targetV, waitTimeForTriggerClose);
        motors.triggerClose();

        // starting shoot third one
        targetV = targetV - 5;
        reachTargetVelocity(targetV, waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot third
        Logging.log("launcher velocity for #3 one: %f.", motors.getLaunchVelocity());
        velocityRampDown(targetV, waitTimeForTriggerClose + waitTimeForTriggerOpen); // waiting more time for the third one.
    }


    private void velocityRampDown(double shootSpeed, int msec) {
        double startTime = runtime.milliseconds();
        boolean artifactReached = false;

        while (!artifactReached && ((runtime.milliseconds() - startTime) < msec)) {
            double currentVel = motors.launcherAverageVelocity(12);
            artifactReached = (currentVel < shootSpeed * 0.87); // when speed reduced to 87%
        }
        Logging.log(" ** Total ramp down duration = %.2f", runtime.milliseconds() - startTime);
    }

    private void reachTargetVelocity(double targetVel, int msec) {
        double startTime = runtime.milliseconds();
        boolean rampedUp = false;
        motors.setLauncherVelocity(targetVel); // update target velocity
        while (!rampedUp && ((runtime.milliseconds() - startTime) < msec)) {
            double currentVel = motors.launcherAverageVelocity(12);
            rampedUp = (currentVel > targetVel * 0.95); // when speed reduced to 85%
        }
        Logging.log(" #### Total ramp up duration = %.2f", runtime.milliseconds() - startTime);
    }


    // action to start launcher motor
    private class startLauncherAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Logging.log("start launcher motor.");
            motors.startLauncherFar();
            return false;
        }
    }
}
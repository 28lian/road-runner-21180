package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Near 2026", group = "Concept")

@Disabled
public class _AutoNearOld2026 extends LinearOpMode {
    // get the software-hardware links ready
    private MecanumDrive drive;
    private intakeUnit2026 motors;
    private Colored patternDetector;

    private int leftOrRight = 1; // 1 for blue, -1 for red
    private double detectedPattern; // limelight detected pattern

    private Pose2d StartPose; // where the robot is placed at the start
    private Vector2d shootPos; // where the robot should shoot
    private double shootHeading; //the direction the robot shoot in

    @Override
    public void runOpMode() {
        // connect the hardware map to color discrimination system and prepare launcher, intake, and trigger
        patternDetector = new Colored(hardwareMap);
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        // set the starting position at (6 HALFMATS - HALF OF ROBOT LENGTH, HALF OF ROBOT WIDTH [sign depends on sign]) and with heading in reverse
        StartPose = new Pose2d(
                (6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH),
                (leftOrRight * Params.CHASSIS_HALF_WIDTH), Math.toRadians(180.0)
        );

        // define shoot position at (HALF_MAT, HALF_MAT), (HALF_MAT, -HALF_MAT), calculate shoot angle based on location x, y
        double shootPosX = 1 * Params.HALF_MAT;
        double shootPosY = leftOrRight * Params.HALF_MAT;
        shootHeading = Math.toRadians(180) + Math.atan2(6 * Params.HALF_MAT - shootPosY, 6 * Params.HALF_MAT - shootPosX);
        shootPos = new Vector2d(shootPosX, shootPosY);

        // set up driving system
        drive = new MecanumDrive(hardwareMap, StartPose);
        // before it even starts, try to detect the AprilTag (for debug purposes/safety)
//        while (!isStarted()) {
//            sleep(10);
//            telemetry.addLine("Initialized. Waiting for start...");
//            detectAprilTeg();
//            telemetry.addData("limelight", "Detected Pattern = %f", detectedPattern);
//            telemetry.update();
//        }
        waitForStart();
        if (opModeIsActive()) {
            run_auto();
        }
    }

    private void run_auto() {
        motors.triggerClose();
//        detectedPattern = 21; //set default pattern if limelight can't detect
        // Detect April Tag while Moving backwards to detect position
        // Then turn to shoot position angle.
        updateProfileAccel(false);
        Action leg1 = drive.actionBuilder(drive.localizer.getPose())
                .afterDisp(3 * Params.HALF_MAT, new limeLightCamera()) // start detecting after moving 20 inches
                .afterDisp(3.1 * Params.HALF_MAT, new startLauncherAction())
                .strafeToConstantHeading(shootPos)
                .turnTo(shootHeading)
                .build();

        // detect April Tag while moving backwards to shoot position.
        Actions.runBlocking(leg1);

        // shoot preload artifacts
        shootArtifacts();
        VelConstraint baseVelConstraint = (robotPose, _path, _disp) -> {
            return 20.0;
        };
        // moving to detect patten row of artifacts to pick up
        for (int pickupIndex = 0; pickupIndex < 3; pickupIndex++ ) {
            Vector2d pickupPos;
            Vector2d pickupEndPos;

            pickupPos = rowChoose((detectedPattern + pickupIndex) % 3);

            pickupEndPos = new Vector2d(pickupPos.x, pickupPos.y + leftOrRight * (2 + ((pickupPos.x>0)? 0 : 1))* Params.HALF_MAT );

            if ((detectedPattern + pickupIndex) % 3 > 0) {
                Action actIntake1 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(pickupPos, Math.toRadians(90.0))
                        .afterTime(0.001, new startIntakeAction())
                        .strafeToConstantHeading(pickupEndPos, baseVelConstraint) // picking up artifacts
                        .strafeToConstantHeading(pickupPos)
                        .afterTime(0.001, new startLauncherAction())
                        .strafeToLinearHeading(shootPos, shootHeading)
                        .build();
                Actions.runBlocking(actIntake1);
            } else {
                Action actIntake1 = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(pickupPos, Math.toRadians(90.0))
                        .afterTime(0.001, new startIntakeAction())
                        .strafeToConstantHeading(pickupEndPos, baseVelConstraint) // picking up artifacts
                        .afterTime(0.001, new startLauncherAction())
                        .strafeToLinearHeading(shootPos, shootHeading)
                        .build();
                Actions.runBlocking(actIntake1);
            }
            // shoot first picked up artifacts
            shootArtifacts();

            // stop intake motor
            motors.stopIntake();
        }
    }

    private void shootArtifacts() {
        int waitTimeForTriggerClose = 300;
        int waitTimeForTriggerOpen = 950;
        Logging.log("start shooting.");
        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < 0.1) {
            Logging.log("start launcher motor since it is stopped.");
            motors.startLauncher();
            sleep(waitTimeForTriggerOpen + 500); // waiting time for launcher motor ramp up
        }

        motors.triggerOpen(); // shoot first
        sleep(waitTimeForTriggerClose);
        motors.triggerClose(); //close trigger to wait launcher motor speed up after first launching

        motors.startIntake(); // start intake motor to move 3rd artifacts into launcher
        sleep(waitTimeForTriggerOpen);// waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot second
        sleep(waitTimeForTriggerClose);

        motors.triggerClose();
        sleep(waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen();  // shoot third
        sleep(waitTimeForTriggerClose + 150);

        motors.triggerClose();
        //motors.stopIntake();
        motors.stopLauncher();
    }

    private void sortArtifacts(int pattern) {
        switch (pattern) {
            case 21:
                telemetry.addLine("Sorting: GPP");
                break;
            case 22:
                telemetry.addLine("Sorting: PGP");
                break;
            case 23:
                telemetry.addLine("Sorting: PPG");
                break;
            default:
                telemetry.addLine("Sorting: Unknown");
                break;
        }
        telemetry.update();
    }

    // function that chooses the right row based on detected pattern, returns a Vector2d
    private Vector2d rowChoose(double rownumber) {
        return new Vector2d(
                (1 - (rownumber * 2)) * Params.HALF_MAT,
                leftOrRight * (2 * Params.HALF_MAT + Params.CHASSIS_HALF_LENGTH)
        );
    }

    private class limeLightCamera implements Action {
        @Override
        // initialized limelight to detect pattern
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (int i = 0; i < 30; i++) { // up to ~0.3s
                detectedPattern = patternDetector.returnId(false);
                Logging.log("pattern  = %f", detectedPattern);
                telemetry.addData("line light","Detected Pattern = %f", detectedPattern);
                telemetry.update();
                if ((detectedPattern > 20) && (detectedPattern < 24)) {
                    return false;
                }
                sleep(1);
            }
            return false;
        }
    }

    private class startLauncherAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Logging.log("start launcher motor.");
            motors.startLauncher();
            return false;
        }
    }

    private class startIntakeAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Logging.log("start intake motor.");
            motors.startIntake();
            return false;
        }
    }

    private void updateProfileAccel(boolean slowMode) {
        if (slowMode) {
            MecanumDrive.PARAMS.maxWheelVel = 65;
            MecanumDrive.PARAMS.maxProfileAccel = 50;
            MecanumDrive.PARAMS.minProfileAccel = -40;
        } else {
            MecanumDrive.PARAMS.maxWheelVel = 70;
            MecanumDrive.PARAMS.maxProfileAccel = 70;
            MecanumDrive.PARAMS.minProfileAccel = -70;
        }
    }
}

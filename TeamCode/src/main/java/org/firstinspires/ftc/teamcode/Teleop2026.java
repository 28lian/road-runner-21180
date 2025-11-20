/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 /*
  * PID controller and IMU codes are copied from
  * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
  */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Hardware config:
 *      imu on control Hub:
 *          "imu"
 *
 *      Four drive motors:
 *          "FrontLeft"
 *          "BackLeft"
 *          "BackRight"
 *          "FrontRight"
 *
 *      Tow arm, wrist motors:
 *          "Arm"
 *          "Wrist"
 *
 *      Tow servo motors:
 *          "Knuckle"
 *          "Finger"
 */

@TeleOp(name="Teleop - 2026", group="Concept")
public class Teleop2026 extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    MecanumDrive drive;

    // initialize limelight
    private Colored patternDetector;

    //claw and arm unit
    private intakeUnit2026 motors;

    boolean debugFlag = true;
    GamePadButtons2026 gpButtons = new GamePadButtons2026();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        patternDetector = new Colored(hardwareMap);

        drive = new MecanumDrive(hardwareMap, Params.currentPose);
        Logging.log("param.currentPose heading = %s", Params.currentPose.heading.log());

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        //set shoot position
        int leftOrRight = 1;
        double shootPosX = 1 * Params.HALF_MAT;
        double shootPosY = leftOrRight * Params.HALF_MAT;
        double shootHeading = Math.toRadians(180) + Math.atan2(leftOrRight * (6 * Params.HALF_MAT - Math.abs(shootPosY)), 6 * Params.HALF_MAT - shootPosX);
        Vector2d shootPos = new Vector2d(shootPosX, shootPosY);

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //preset positions used for teleop commands
        while (!isStarted()) {
            double[] patternPos = patternDetector.returnPosition();
            if (patternPos.length >= 2) {
                telemetry.addData("Pattern ", "X: %.1f Y: %.1f", patternPos[0], patternPos[1]);
                telemetry.addData("LL", "%s", patternPos.toString());
            }
            else
            {
                telemetry.addData("no pattern detected", 0);
            }
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start.");

        telemetry.update();
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad1 buttons
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);

            // drive speed control
            double maxDrivePower;
            if (gpButtons.speedUp) {
                maxDrivePower = Params.POWER_HIGH;
            } else if (gpButtons.speedDown) {
                maxDrivePower = Params.POWER_LOW;
            } else {
                maxDrivePower = Params.POWER_NORMAL;
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * maxDrivePower,
                            -gpButtons.robotStrafe * maxDrivePower
                    ),
                    -gpButtons.robotTurn * maxDrivePower
            ));
            drive.updatePoseEstimate(); // update dire pose.

//            if (gpButtons.alignShootPos) {
//                Actions.runBlocking(
//                        drive.actionBuilder(new AutoTest().newStartPose)
//                                .strafeToLinearHeading(new AutoTest().shootPos.position, new AutoTest().shootPos.heading)
//                                .build()
//                );
//            }
//
//            if (gpButtons.autoPark) {
//                Actions.runBlocking(
//                        drive.actionBuilder(new AutoTest().newStartPose)
//                                .strafeToLinearHeading(new Vector2d(-4 * Params.HALF_MAT, 2 * Params.HALF_MAT), Math.toRadians(90))
//                                .build()
//                );
//            }

            // launch actions
            if (gpButtons.launch) {
                motors.startLauncher();
            }
            if (gpButtons.launchOff) {
                motors.stopLauncher();
            }

            // in case there are 4 artifacts on robot.
            // shoot one out
            if (gpButtons.launchOneNear) {
                int rampUpTime = 600;
                int waitTimeForTriggerClose = 1000;
                double launchVelocity = motors.launchSpeedNear;
                motors.startLauncher();
                motors.startIntake();
                reachTargetVelocity(launchVelocity, rampUpTime);
                motors.triggerOpen(); // shoot first
                checkingVelocityRampDown(waitTimeForTriggerClose);
                motors.triggerClose();
            }

            // shoot one out for Far launching
            if (gpButtons.launchOneFar) {
                int rampUpTime = 600;
                int waitTimeForTriggerClose = 1000;
                double launchVelocity = motors.launchSpeedFar;
                motors.startLauncherFar();
                motors.startIntake();
                reachTargetVelocity(launchVelocity, rampUpTime);
                motors.triggerOpen(); // shoot first
                checkingVelocityRampDown(waitTimeForTriggerClose);
                motors.triggerClose();
            }

            // shoot one out for Far launching
            if (gpButtons.dumpOne) {
                int rampUpTime = 400;
                int waitTimeForTriggerClose = 400;
                double launchVelocity = motors.launchSpeedDump;
                motors.setLauncherVelocity(launchVelocity);
                reachTargetVelocity(launchVelocity, rampUpTime);
                motors.triggerOpen(); // shoot first
                checkingVelocityRampDown(waitTimeForTriggerClose);
                motors.triggerClose();
            }

            // intake actions
            if (gpButtons.intakeOn) {
                motors.startIntake();
            }

            if (gpButtons.intakeOff) {
                motors.stopIntake();
            }

            // trigger servo actions
            if (gpButtons.triggerOpen) {
                motors.triggerOpen();
            }

            if (gpButtons.triggerClose) {
                motors.triggerClose();
            }

            if (gpButtons.launchArtifacts) {
                shootArtifacts(false);
            }

            if (gpButtons.launchArtifactsFar) {
                // turn for heading correction before shooting
                Actions.runBlocking(
                        drive.actionBuilder(drive.localizer.getPose())
                                .turnTo(Math.toRadians(motors.launchDegreeFar * Params.leftOrRight))
                                .build()
                );

                shootArtifacts(true);
            }

            // TODO : not implemented correctly yet
//            if(gpButtons.autoLaunchPos) {
//                // move angle to launch using limelight return position (which returns degrees x and degrees y of the pattern)
//                // telemetry
//                double[] patternPos = patternDetector.returnPosition();
//                if (patternPos.length >= 2) {
//                    double angleToTurn = patternPos[0];
////                    Pose3D botpose = patternDetector.returnPositionbeta();
////                    Actions.runBlocking(
////                            drive.actionBuilder(new Pose2d(botpose.getPosition().x, botpose.getPosition().y, botpose.getOrientation().getYaw()))
////                                    .strafeToLinearHeading(new Vector2d(1*Params.HALF_MAT, 1*Params.HALF_MAT), Math.toRadians(135))
////                                    .build()
////                    );
//                    if (angleToTurn > 0) {
//                        Actions.runBlocking(
//                                drive.actionBuilder(new Pose2d(0,0,0))
//                                        .strafeToLinearHeading(new Vector2d(0,0.00000000000000000000000001), 3.141592653589 * angleToTurn / 180)
//                                        .build()
//                        );
//                    } else if (angleToTurn < 0) {
//                        Actions.runBlocking(
//                                drive.actionBuilder(new Pose2d(0,0,0))
//                                        .strafeToLinearHeading(new Vector2d(0,0.00000000000000000000000001), -3.141592653589 * Math.abs(angleToTurn) / 180)
//                                        .build()
//                        );
//                    }
//                }
//            }

            if (debugFlag) {

                // display trigger servo position for testing purpose.
                telemetry.addData("trigger servo", "position = %.3f", motors.getTriggerPosition());
                telemetry.addData("launcher motor", "power = %.3f", motors.getLauncherPower());
                telemetry.addData("launcher motor", "velocity = %.3f", motors.getLaunchVelocity());

                /*
                Logging.log("launcher motor velocity : %.1f. power = %.3f", motors.getLaunchVelocity(), motors.getLauncherPower());

                double[] patternPos = patternDetector.returnPosition();
                if (patternPos.length >= 2) {
                    telemetry.addData("Pattern ", "X: %.1f Y: %.1f", patternPos[0], patternPos[1]);
                } else {
                    telemetry.addData("no pattern detected", 0);
                }

                 */

                telemetry.addData("heading", " %.3f", Math.toDegrees(drive.localizer.getPose().heading.log()));
                telemetry.addData("location", " %s", drive.localizer.getPose().position.toString());
                telemetry.addData(" --- ", " --- ");
                telemetry.update(); // update message at the end of while loop

            }

        }

        // The motor stop on their own but power is still applied. Turn off motor.
    }

    public void shootArtifacts(boolean farLaunch) {
        int waitTimeForTriggerClose = 1000;
        int waitTimeForTriggerOpen = 600;
        int rampUpTime = 800;
        double launchVelocity = motors.launchSpeedNear;
        if (farLaunch) {
            launchVelocity = motors.launchSpeedFar;
        }

        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < 0.4) {
            if (farLaunch) {
                motors.startLauncherFar(); // far power
            } else {
                motors.startLauncher(); // near
            }
        }
        // launcher is started but with near launching power
        else if ((motors.getLaunchVelocity() < motors.launchSpeedFar) && farLaunch) {
            motors.startLauncherFar();
        }

        reachTargetVelocity(launchVelocity, rampUpTime);
        motors.triggerOpen(); // shoot first
        checkingVelocityRampDown(waitTimeForTriggerClose);
        motors.triggerClose(); //close trigger to wait launcher motor speed up after first launching

        // start shooting 2nd one
        launchVelocity -= 4; // reduce a little bit.
        motors.startIntake(); // start intake motor to move 3rd artifacts into launcher
        reachTargetVelocity(launchVelocity, waitTimeForTriggerOpen);// waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot second
        checkingVelocityRampDown(waitTimeForTriggerClose);
        motors.triggerClose();

        // start shooting 3rd one
        launchVelocity -= 1; // reduce a little bit.
        reachTargetVelocity(launchVelocity, waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen();  // shoot third
        checkingVelocityRampDown(waitTimeForTriggerClose);

        motors.triggerClose();
        motors.stopIntake();
        motors.stopLauncher();
    }

    // sleep for other motors than driving motors
    private void sleepWithDriving(int msecond) {
        double startTime = runtime.milliseconds();
        while ((runtime.milliseconds() - startTime) < msecond) {
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * Params.POWER_LOW / 2.0,
                            -gpButtons.robotStrafe * Params.POWER_LOW / 2.0
                    ),
                    -gpButtons.robotTurn * Params.POWER_LOW / 2.0
            ));
        }
    }

    private void checkingVelocityRampDown(int msecond) {
        double startTime = runtime.milliseconds();

        boolean artifactReached = false;
        double stableVelocity;
        stableVelocity = motors.launcherAverageVelocity(20);

        while (!artifactReached && ((runtime.milliseconds() - startTime) < msecond)) {
            double currentVel = motors.launcherAverageVelocity(20);
            artifactReached = (currentVel < stableVelocity * 0.87); // when speed reduced to 87%

            gpButtons.checkGamepadButtons(gamepad1, gamepad2);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * Params.POWER_LOW / 2.0,
                            -gpButtons.robotStrafe * Params.POWER_LOW / 2.0
                    ),
                    -gpButtons.robotTurn * Params.POWER_LOW / 2.0
            ));
        }
    }

    private void reachTargetVelocity(double targetVel, int msec) {
        double startTime = runtime.milliseconds();
        boolean rampedUp = false;
        motors.setLauncherVelocity(targetVel); // update target velocity

        while (!rampedUp && ((runtime.milliseconds() - startTime) < msec)) {
            double currentVel = motors.launcherAverageVelocity(20);
            rampedUp = (currentVel > targetVel * 0.90); // when speed reduced to 85%
            Logging.log("launcher motor average velocity : %.1f.", currentVel);
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * Params.POWER_LOW / 1.5,
                            -gpButtons.robotStrafe * Params.POWER_LOW / 1.5
                    ),
                    -gpButtons.robotTurn * Params.POWER_LOW / 1.5
            ));
        }
        Logging.log("Total waiting duration = %.2f", runtime.milliseconds() - startTime);
    }
}

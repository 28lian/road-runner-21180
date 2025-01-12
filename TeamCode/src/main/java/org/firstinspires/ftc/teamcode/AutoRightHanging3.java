/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * PID controller and IMU codes are copied from
 * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
 *      Servo motors:
 *          "FingerServo"
 *          "WristServo"
 *          "SwitchServo"
 *
 *      One cameras:
 *          "Webcam 1"
 */

@Autonomous(name=" B - 4 specimens", group="Concept")
//@Disabled
public class AutoRightHanging3 extends AutoRightHanging2 {

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        setStartPoses(leftOrRight);

        // init drive with road runner
        drive = new MecanumDrive(hardwareMap, newStartPose);

        // reset imu at the beginning of autonomous
        imu = drive.lazyImu.get();
        imu.resetYaw();
        Params.imuReseted = true;
        double imu_heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        Params.startPose = newStartPose; // init storage pose.
        Params.currentPose = newStartPose;; // init storage pose

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist", "Knuckle","Finger");

        // set RunToPosition mode and set power for motors.
        intake.setWristModeRunToPosition(intake.getWristPosition());
        intake.setArmModeRunToPosition(intake.getArmPosition());

        intake.setIntakeAutoInit();

        // you can use this as a regular DistanceSensor.
        distSensorB = hardwareMap.get(DistanceSensor.class, "distanceB");
        distSensorF = hardwareMap.get(DistanceSensor.class, "distanceF");

        while (!isStarted()) {
            sleep(10);
            telemetry.addData("Status", " IMU heading = %2f", imu_heading + 180.0);

            telemetry.addData( "FTC 2024 - ", "Wait for starting ");

            telemetry.addData("deviceName", distSensorB.getDeviceName() );
            telemetry.addData("back range", " = %2f", distSensorB.getDistance(DistanceUnit.INCH));


            telemetry.addData("deviceName", distSensorF.getDeviceName() );
            telemetry.addData("front range", " = %2f", distSensorF.getDistance(DistanceUnit.INCH));

            telemetry.addData("Arm", "position = %d", intake.getArmPosition());

            telemetry.addData(" ---- ", " ---  ");

            telemetry.addData("Arm calibration - -    ", Params.armCalibrated? "Pass!" :" Failed !!!!!");

            telemetry.update();
        }

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        waitForStart();

        Logging.log("Before start wrist pos: %s", intake.getWristPosition());
        Logging.log("Before start finger pos: %2f", intake.getFingerPosition());
        Logging.log("Before start arm pos: %s", intake.getArmPosition());

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            if (Params.armCalibrated) {
                autonomousCore();

                Logging.log("Autonomous time - total Run Time: " + runtime);
            } else {
                telemetry.addData("Arm calibration: ---", "need to be done before starting!");
                telemetry.update();
                sleep(4000);
            }
            Params.currentPose = drive.pose; // save current position
        }

    }

    private void autonomousCore() {

        Logging.log("Status - Start auto core");
        Logging.log("Auto start wrist pos: %s", intake.getWristPosition());

        //new stuff for 2024-2025 season
        //hang specimen
        Vector2d firstHighChamberPos = new Vector2d(-3.0 * Params.HALF_MAT, newStartPose.position.y);

        //grab
        Vector2d firstSamplePosition = new Vector2d(- 3.15 * Params.HALF_MAT, - 4.1 * Params.HALF_MAT);

        // adjust x for second sample a little bit according to testing
        Vector2d secondSamplePosition = new Vector2d(- 3.20 * Params.HALF_MAT, firstSamplePosition.y - 10.5);

        Vector2d obsZone = new Vector2d(- 4.3 * Params.HALF_MAT, - 3.5 * Params.HALF_MAT);

        updateProfileAccel(false); // fast mode
        if (leftOrRight == 1) { // right side auto
            //Go to position for arm flip and hang on high chamber
            Logging.log(" Start position: X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
            Actions.runBlocking(
                    drive.actionBuilder(newStartPose)
                            .afterTime(0.01, new armToReadyHangAct())
                            .strafeTo(firstHighChamberPos)
                            .build()
            );

            // adjust robot position by distance sensor. Temp comment out for saving time
            adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorB, drive);

            // Adjust hanging specimen position.x according to the preload specimen hanging position
            // after adjusted by distance sensor
            Params.hangingSpecimenX = drive.pose.position.x; // restore hang position for teleop.
            hangSpecimenPos = new Vector2d(Params.hangingSpecimenX - 2.5, firstHighChamberPos.y);
            Logging.log("Distance sensor reading for hanging preload specimen: %2f", distSensorB.getDistance(DistanceUnit.INCH));
            Logging.log(" Preload hang Specimen Pos: X position = %2f, defined pos X = %2f", drive.pose.position.x, firstHighChamberPos.x);
            Logging.log("hanging # 0 specimen X position = %2f", Params.hangingSpecimenX);

            //hanging action
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
            sleep(400);

            //release specimen and lower arm to clear high chamber
            intake.setFingerPosition(intake.FINGER_OPEN);
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);
            intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);

            sleep(200);
            //Go to pick up first sample on mat
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(1.4, new armToPickUpPos()) // lower arm during spline moving
                            .strafeToLinearHeading(firstSamplePosition, newStartPose.heading)//go to first sample position
                            .turnTo(headingAngleCorrection) // fine adjust heading
                            .build()
            );
            Logging.log("pickup first sample: dead wheel heading = %2f", Math.toDegrees(drive.pose.heading.log()));
            Logging.log("pickup first sample: imu heading = %2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 180);


            intake.fingerServoClose();
            sleep(150);
            intake.setKnucklePosition(intake.KNUCKLE_SIZE_CONSTRAINT);
            intake.setArmPosition(intake.ARM_POS_DROP_SAMPLE);
            intake.setWristPosition(intake.WRIST_POS_NEUTRAL);

            // strafe to second sample during flip arm to drop first sample
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(secondSamplePosition, newStartPose.heading)//go to second sample position
                            .turnTo(headingAngleCorrection)
                            .build()
            );
            intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SAMPLE_BACK);
            sleep(400); // waiting arm reaching position to drop off first sample.
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE_BACK);
            intake.setWristPosition(intake.WRIST_BACK);
            intake.fingerServoOpen();
            sleep(200);
            intake.setKnucklePosition(intake.KNUCKLE_SIZE_CONSTRAINT); // flip knuckle before lift arm due to size limitation

            sleep(1000); // wait arm flip back to pickup second sample
            intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SAMPLE_BACK); // flip knuckle before lift arm due to size limitation
            sleep(400);
            intake.fingerServoClose();
            Logging.log("pickup second sample: dead wheel heading = %2f", Math.toDegrees(drive.pose.heading.log()));
            Logging.log("pickup second sample: imu heading = %2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 180);

            sleep(150); // wait finger close to grab second sample
            intake.setKnucklePosition(intake.KNUCKLE_SIZE_CONSTRAINT); // flip knuckle before lift arm due to size limitation
            sleep(150);
           // moving during arm flip for wall case, starting flip arm here
            intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN_WALL); // flip arm to grab specimen position before drop off second sample
            intake.setWristPosition(intake.WRIST_POS_NEUTRAL);
            sleep(800);
            
            // pick up specimen from wall, loop to hang specimen
            for (int j = 1; j < 4; j++) {

                Logging.log(" Start moving to wall to pickup # %s specimen. ", j);

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.1, new intakeAct(intake.ARM_POS_GRAB_SPECIMEN_WALL, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_PICKUP_SPECIMEN_WALL, Params.NO_CATION))
                                .strafeToLinearHeading(pickupSpecimenLineup, newStartPose.heading) // line up
                                //.turnTo(headingAngleCorrection) // fine correct heading
                                .build()
                );
                intake.fingerServoOpen(); // drop off the second sample
                // using distance sensor to move robot to correct position for pickup specimen from wall
                adjustPosByDistanceSensor(Params.SPECIMEN_PICKUP_DIST, distSensorF, drive);

                Params.pickupSpecimenX = drive.pose.position.x; // restore X position for teleop.
                Params.pickupSpecimenLineupX = drive.pose.position.x + 3.0; // leave 3.0 inch gap to avoid hitting the specimen
                // adjust wall pickup position.x according to distance sensor to speedup next pickup.
                pickupSpecimenLineup = new Vector2d(Params.pickupSpecimenLineupX, -3.8 * Params.HALF_MAT);
                Logging.log(" After adjust of pickupSpecimenLineup: X position = %2f", pickupSpecimenLineup.x);

                // start picking up actions
                intake.fingerServoClose();
                sleep(150); // waiting finger close
                intake.setKnucklePosition(intake.KNUCKLE_POS_LIFT_FROM_WALL);
                //sleep(100); // wait knuckle lift the specimen

                Logging.log(" Start moving to high chamber to hang # %s specimen. ", j);
                // strafe to high chamber for hanging specimen
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                // flip arm to high chamber position, back knuckle to avoid hitting chamber during strafing
                                .afterTime(0.1, new intakeAct(intake.ARM_POS_HIGH_CHAMBER_READY, intake.WRIST_BACK, Params.NO_CATION, Params.NO_CATION))
                                // shift 1.5 inch for each specimen on high chamber
                                .strafeToLinearHeading(new Vector2d(hangSpecimenPos.x, hangSpecimenPos.y - 1.3 * j), newStartPose.heading)
                                // get knuckle ready for hanging
                                .afterTime(0.001, new intakeAct(Params.NO_CATION, Params.NO_CATION, intake.KNUCKLE_POS_HIGH_CHAMBER, Params.NO_CATION))
                                .turnTo(headingAngleCorrection) // fine correct heading
                                .build()
                );

                //adjust pos using distance sensor
                adjustPosByDistanceSensor(Params.HIGH_CHAMBER_DIST, distSensorB, drive);
                // restore hang position for teleop.
                // this one should be more accurate than the preload one, due to the same pathway for teleop.
                Params.hangingSpecimenX = drive.pose.position.x;
                hangSpecimenPos = new Vector2d(Params.hangingSpecimenX - 1.0, firstHighChamberPos.y);
                Logging.log("hanging # %d specimen X position = %2f", j, Params.hangingSpecimenX);

                //hang specimen
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
                sleep(400);
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_MOVING_SPECIMEN);
                intake.fingerServoOpen(); // release specimen
                intake.setKnucklePosition(intake.KNUCKLE_POS_AWAY_FROM_SUBMERSIBLE);
                sleep(200);
            }

            Logging.log("hanging specimen complete, start parking");

            // parking
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.2, new intakeAct(intake.ARM_POS_OBZ_PARKING, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_LIFT_FROM_WALL, intake.FINGER_CLOSE))
                            .strafeToLinearHeading(obsZone, Math.toRadians(-130))
                            .build()
            );
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
        }
    } // end auto_core()

    
}
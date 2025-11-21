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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

@TeleOp(name="PID Tune", group="Concept")
@Disabled
public class PIDTune extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    //claw and arm unit
    public  DcMotorEx motor;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        motor = hardwareMap.get(DcMotorEx.class, "launcher");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // update launcher motor PID for quick ramp up and keep the speed
        double p = 100.0;
        double i = 3.5;
        double d = 0.2;
        double f = 0.00361;
        motor.setVelocityPIDFCoefficients(p, i, d, f);

        boolean dumpLog = false;
        PIDFCoefficients  pid = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        double vel = 195;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start.");

        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // launch actions
            if ((motor.getVelocity(AngleUnit.DEGREES) < 0.1) && (gamepad1.left_bumper)) {

                runtime.reset();

                dumpLog = true;
                Logging.log("start motor with target velocity = %.1f", vel);
                pid = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

                Logging.log("p-i-d input p = %.5f, i = %.5f, d = %.5f, f = %.5f", pid.p, pid.i, pid.d, pid.f);

                motor.setVelocity(vel, AngleUnit.DEGREES);
            }

            if(gamepad1.left_stick_x > 0) {
                vel += 1;
            }

            if (gamepad1.left_stick_x < 0) {
                vel -= 1;
            }

            if (gamepad1.right_bumper) {
                motor.setPower(0);
                dumpLog = false;
            }

            if (gamepad1.b) {
                motor.setVelocityPIDFCoefficients(p, i, d, f);
            }

            if (gamepad1.a) {
                p = 100.0;
                i = 3.5;
                d = 0.2;
                f = 0.00361;
            }

            if (gamepad1.dpad_up) {
                p += 0.1;
            }

            if(gamepad1.dpad_right) {
                i += 0.01;
            }

            if(gamepad1.x) {
                d += 0.001;
            }


            if (gamepad1.dpad_down) {
                p -= 0.1;
            }

            if(gamepad1.dpad_left) {
                i -= 0.01;
            }

            if(gamepad1.y) {
                d -= 0.001;
            }

            if (gamepad1.back) {
                motor.setVelocityPIDFCoefficients(p, 1.0, d, f);
                pid = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                Logging.log("p-i-d input p = %.5f, i = %.5f, d = %.5f, f = %.5f", pid.p, pid.i, pid.d, pid.f);

            }

            telemetry.addData("target speed in degree", "= %.1f", vel);

            telemetry.addData("p-i-d input", "p = %.5f, i = %.5f, d = %.5f, f = %.5f", p, i, d, f);

            telemetry.addData("current PID", "p = %.5f, i = %.5f, d = %.5f, f = %.5f", pid.p, pid.i, pid.d, pid.f);

            telemetry.addData("motor", "power = %.3f", motor.getPower());
            telemetry.addData("current speed in degree", " = %.3f", motor.getVelocity(AngleUnit.DEGREES));

            telemetry.addData(" --- ", " --- ");
            telemetry.update(); // update message at the end of while loop

            if (dumpLog) {
                Logging.log("motor: t, v, p = %5.1f, %.1f, %.3f", runtime.milliseconds(), motor.getVelocity(AngleUnit.DEGREES), motor.getPower());
            }
        }
        // The motor stop on their own but power is still applied. Turn off motor.
    }

}

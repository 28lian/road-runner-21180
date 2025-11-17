package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Game pad buttons design:
 * Game Pad1:
 *  Left stick:
 *      left-right: strafe robot
 *      up-down:    drive root
 *  Right stick:
 *      left-right: turn robot
 *  dpad:
 *      left:       robot movement speed down
 *      right:      robot movement speed up
 *      up:         normal auto pickup the 3rd cone from cone stack
 *      down:       normal auto pickup the 4th and 5th cone from cone stack
 *  X:              robot movement speed down (same as dpad_left)
 *  B:              robot movement speed down (same as dpad_right)
 *  Left Bumper:    normal auto pickup cone from ground
 *  Right bumper:   normal auto drop off cone
 *  Left trigger:   auto pick up cone from cone base then move to high junction, slider lifted to high junction
 *  Right trigger:  auto drop off cone on high junction then move to cone base
 *  Y:              Teapot function - drop off cone on high junction then pick up cone from base, then move to high junction
 *  Back:           Back to cone base just at the beginning of Teleop
 *  A:              open claw to drop off cone manually.*
 * Game pad2:
 *  dpad:
 *      Up:         close the claw to pick up cone
 *      Down:       open claw to drop off cone manually
 *  Right stick:
 *                  left-right: manually control slider up and down
 *  X:              move slider to wall position
 *  A:              move slider to low junction position
 *  B:              move slider to medium junction position
 *  Y:              move slider to high junction position
 *  Right bumper:   move slider to ground junction position
 *  Right trigger:  move slider to ground position
 *  Left stick:
 *      left-right: manually control arm position
 *      up:         move arm to pick up position
 *      down:       move arm to drop off position
 *
 */
public class GamePadButtons2026 {
    //game pad setting
    public float robotDrive;
    public float robotStrafe;
    public float robotTurn;
    public boolean speedDown;
    public boolean speedUp;
    public boolean alignShootPos;
    public boolean autoPark;
    public boolean launch;
    public boolean launchOneNear;
    public boolean launchOneFar;
    public boolean launchOff;
    public boolean intakeOn;
    public boolean intakeOff;
    public boolean autoLaunchPos;
    public boolean triggerOpen;
    public boolean triggerClose;
    public boolean launchArtifacts;
    public boolean launchArtifactsFar;

    public void checkGamepadButtons(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        //game pad 1 buttons

        //driving
        /*
        robotDrive = gamepad1.left_stick_y;
        robotStrafe = gamepad1.left_stick_x;
        robotTurn = gamepad1.right_stick_x;
         */

        robotDrive = gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y;
        robotStrafe = gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x;
        robotTurn = gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x;

        //speed controls
        speedUp = gamepad1.right_bumper;
        speedDown = gamepad1.left_bumper;

        launch = gamepad2.left_bumper; // temp button can change later
        launchOneNear = gamepad2.dpad_up;
        launchOneFar = gamepad2.dpad_down;

        launchOff = gamepad2.right_bumper; // temp button can change later

        // intake buttons
        intakeOn = (gamepad1.left_trigger > 0.1) || (gamepad2.left_trigger > 0.1);
        intakeOff = (gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1);

        // trigger servo buttons
        triggerOpen = gamepad1.y || gamepad2.y;
        triggerClose = gamepad1.x || gamepad2.x;

        // launcher buttons
        launchArtifacts = gamepad1.dpad_left || gamepad2.dpad_left;
        launchArtifactsFar = gamepad1.dpad_right || gamepad2.dpad_right;

        autoLaunchPos = gamepad2.a;

        alignShootPos = gamepad1.a; // temp button can chang later
        autoPark = gamepad1.b; // temp button can change later
    }
}

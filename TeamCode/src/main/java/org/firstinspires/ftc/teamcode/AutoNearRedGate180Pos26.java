package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Near Red Gate Pos180", group = "Concept")
public class AutoNearRedGate180Pos26 extends AutoNearBlueGate180Pos26 {
    @Override
    public void setSide() {
        leftOrRight = -1;
    }
}
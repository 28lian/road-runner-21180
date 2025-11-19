package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Near Red Gate", group = "Concept")
public class AutoNearRedGate26 extends AutoNearBlueGate26 {
    @Override
    public void setSide() {
        leftOrRight = -1;
    }
}
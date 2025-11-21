package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto Near Red Gate Pos45", group = "Concept")
@Disabled
public class AutoNearRedGate26 extends AutoNearBlueGate26 {
    @Override
    public void setSide() {
        leftOrRight = -1;
    }
}
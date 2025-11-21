package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Far Red shoot pickup", group = "Concept")
public class AutoFarRedPick26 extends AutoFarBluePick26 {
    @Override
    public void setSide() {
        leftOrRight = -1;
    }

    @Override
    public void setPickupRowNum() {
        pickRowNum = 1;
    }
}
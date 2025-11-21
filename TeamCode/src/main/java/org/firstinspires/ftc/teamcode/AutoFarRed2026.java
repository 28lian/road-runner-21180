package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Far Red shoot 2026", group = "Concept")
public class AutoFarRed2026 extends AutoFarBluePick26 {
    @Override
    public void setSide() {
        leftOrRight = -1;
    }

    @Override
    public void setPickupRowNum() {
        pickRowNum = 0;
    }

}
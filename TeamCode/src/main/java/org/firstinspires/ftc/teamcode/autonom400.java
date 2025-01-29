package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class autonom400 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("mda", "am facut" +  "de puncte");
        telemetry.update();
        waitForStart();
}}

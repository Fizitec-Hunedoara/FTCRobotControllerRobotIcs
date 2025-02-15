package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class autonom400 extends LinearOpMode {
    FunctiiDeProgram func = new FunctiiDeProgram(this);
    int i=0;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(func.opMode.opModeIsActive()){
            i++;
            telemetry.addData("mda", "am facut " + i + " de puncte");
            telemetry.update();
            func.kdf_auto(50);
        }
}}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class test_extindere extends LinearOpMode {
    FunctiiDeProgram func = new FunctiiDeProgram(this);
    @Override
    public void runOpMode() throws InterruptedException {
        func.init(hardwareMap,telemetry,false);
        waitForStart();
        func.extindere1.setPosition(0.5);
        func.extindere2.setPosition(0.5);
        long lastTime = System.currentTimeMillis();
        while(lastTime + 1000 > System.currentTimeMillis() && opModeIsActive()) {
            func.extindere1.setPosition(0.2);
            func.extindere2.setPosition(0.9);
        }
        func.kdf_auto(9000);
    }
}

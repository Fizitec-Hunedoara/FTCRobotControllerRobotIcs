package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class test2 extends OpMode {
    FunctiiDeProgram func = new FunctiiDeProgram();
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    @Override
    public void init() {
        func.init(hardwareMap,telemetry,true);
    }
    @Override
    public void loop() {
        if(gamepad2.dpad_left){
            func.pus_in_cos();
        }
        func.articulatorGrabber.setPosition(gamepad2.left_stick_y);
        telemetry.addData("articulator grabber", func.articulatorGrabber.getPosition());
        telemetry.update();
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class articulatie_test extends OpMode {
    FunctiiDeProgram func = new FunctiiDeProgram();
    double articuatorPoz = 0.0;
    @Override
    public void init() {
        func.init(hardwareMap,telemetry,true);
    }
    @Override
    public void loop() {
        if(gamepad2.dpad_up && articuatorPoz < 1){
            articuatorPoz += 0.001;
        }
        else if(gamepad2.dpad_down && articuatorPoz > 0){
            articuatorPoz -= 0.001;
        }
        func.articulatorGrabber.setPosition(articuatorPoz);
        telemetry.addData("articulator grabber", func.articulatorGrabber.getPosition());
        telemetry.update();
    }
}

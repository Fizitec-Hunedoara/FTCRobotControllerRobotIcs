package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
public class articulatie_test extends OpMode {
    FunctiiDeProgram func = new FunctiiDeProgram();
    @Override
    public void init() {
        func.init(hardwareMap,telemetry,true);
    }
    @Override
    public void loop() {
        func.articulatorGrabber.setPosition(gamepad2.left_stick_y);
        telemetry.addData("articulator grabber", func.articulatorGrabber.getPosition());
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Parametri.dslider;
import static org.firstinspires.ftc.teamcode.Parametri.islider;
import static org.firstinspires.ftc.teamcode.Parametri.pslider;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class yupiii extends OpMode {
    FunctiiDeProgram func = new FunctiiDeProgram();
    double pidResult;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0,0,0);
    @Override
    public void init() {
        func.init(hardwareMap,telemetry,true);
        pid.enable();
    }

    @Override
    public void loop() {
        pid.setPID(pslider, islider, dslider);
        if (gamepad2.right_stick_y != 0.0) {
            func.setSliderPower(gamepad2.right_stick_y);
            func.ceva = true;
        }
        else {
            if (func.ceva) {
                func.ceva = false;
                pid.setSetpoint((func.sliderR.getCurrentPosition() + func.sliderL.getCurrentPosition())/2.0);
            }
            else if(!func.atins()){
                pidResult = pid.performPID((func.sliderR.getCurrentPosition() + func.sliderL.getCurrentPosition()) / 2.0);
                func.setSliderPower(pidResult);
            }
            else{
                func.setSliderPower(0);
            }
        }
        telemetry.addData("sliderL:",func.getSliderLPosition());
        telemetry.addData("sliderR:",func.getSliderRPosition());
        telemetry.addData("setPoint:",pid.getSetpoint());
        telemetry.addData("error:",pid.getError());
        telemetry.update();
    }
}

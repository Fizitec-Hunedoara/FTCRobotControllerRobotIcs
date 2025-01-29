package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static java.lang.Math.abs;

import static org.firstinspires.ftc.teamcode.Parametri.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpRosu extends OpMode {
    private final double slowSm = 0.4, fastSm = 1;
    private final double histInterval = 0.2;
    private double pmotorFL, pmotorFR, pmotorBL, pmotorBR;
    double sm = 1.0;
    double y, x, rx;
    double max = 0.0;
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public FunctiiDeProgram func = new FunctiiDeProgram();
    boolean stop = false, puneBara = false;
    boolean apasat = false;
    private boolean rtBool, ltBool, rtBoolLast = false, ltBoolLast = false;
    private long extensorExtensionTriggerTime, extensorRetractionTriggerTime;
    private boolean extensorExtensionOverride = false, extensorRetractionOverride = false;
    public double pidResult;
    public double powIntake = 0.0;
    long lastTime = 0;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        func.init(hardwareMap,telemetry,true);
    }
    public void start() {
        Chassis.start();
        Systems.start();

    }
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                if(gamepad1.left_bumper){
                    sm = slowSm;
                }
                else if (gamepad1.right_bumper){
                    sm = fastSm;
                }

                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x ;
                rx = gamepad1.right_stick_x;

                pmotorFL = y - x - rx;
                pmotorBL = y + x - rx;
                pmotorBR = y + x + rx;
                pmotorFR = y - x + rx;

                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }

                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                func.POWER(pmotorFR * sm, pmotorFL * sm, pmotorBR * sm, pmotorBL * sm);

            }
        }
    });
    private final Thread Systems = new Thread(new Runnable() {

        @Override
        public void run() {
            pid.enable();
            while (!stop) {
                pid.setPID(pslider, islider, dslider);
                if (gamepad2.right_stick_y != 0.0) {
                    func.sliderR.setPower(gamepad2.right_stick_y);
                    func.sliderL.setPower(gamepad2.right_stick_y);
                    func.ceva = true;
                }
                else {
                    if (func.ceva) {
                        func.ceva = false;
                        pid.setSetpoint((func.sliderR.getCurrentPosition() + func.sliderL.getCurrentPosition())/2.0);
                    }
                    else{//else if(!func.atins()){
                        pidResult = pid.performPID((func.sliderR.getCurrentPosition() + func.sliderL.getCurrentPosition()) / 2.0);
                        func.sliderR.setPower(pidResult);
                        func.sliderL.setPower(pidResult);
                    }
                    /*else{
                        func.sliderR.setPower(0);
                        func.sliderL.setPower(0);
                    }*/
                }
                if(gamepad2.a){
                    powIntake = 0.7;
                }
                else if(gamepad2.b){
                    powIntake = 0;
                }
                if(func.colorSensor.blue() > 200){
                    func.intake.setPower(-powIntake);
                }
                else {
                    func.intake.setPower(powIntake);
                }

                if(gamepad2.right_bumper){
                    func.pozGheruta = 0;
                }
                if(gamepad2.left_bumper){
                    func.pozGheruta = 0.154;
                }
                func.gheruta.setPosition(func.pozGheruta);

               /* if(gamepad2.dpad_left && func.pozArticulator < 1){
                    func.pozArticulator += 0.001;
                }
                if(gamepad2.dpad_right && func.pozArticulator > 0){
                    func.pozArticulator -= 0.001;
                }*/

                func.armL.setPosition(func.pozArm);
                func.armR.setPosition(func.pozArm);

                if(gamepad2.dpad_up && !apasat && lastTime + 500 < System.currentTimeMillis()){
                    func.intaketogheara();
                    apasat = true;
                    lastTime = System.currentTimeMillis();
                }
                else if (gamepad2.dpad_up && apasat && lastTime + 300 < System.currentTimeMillis()) {
                    func.ghearatocos();
                    apasat = false;
                    lastTime = System.currentTimeMillis();
                }
                if(gamepad2.dpad_down){
                    func.deschidere();
                    func.pozArm = 0.01;
                    func.pozArticulator = 0.11;
                }
                if(gamepad2.dpad_left){
                    if(!puneBara && lastTime + 500 < System.currentTimeMillis()) {
                        func.gardtogheara();
                        puneBara = true;
                        lastTime = System.currentTimeMillis();
                    }
                    else if(puneBara && lastTime + 500 < System.currentTimeMillis()){
                        puneBara = false;
                        func.ghearapozbara();
                        lastTime = System.currentTimeMillis();
                    }
                }
                if(gamepad2.dpad_right){
                    func.puspebara();
                }
                if(gamepad2.y && func.pozExtindere < 1){
                    func.pozExtindere += 0.005;
                }
                else if(gamepad2.x && func.pozExtindere > 0){
                    func.pozExtindere -= 0.005;
                }
                func.intakePliere.setPosition(func.pozExtindere);
                if(gamepad2.right_trigger > (0.5 + histInterval / 2.0)) {
                    rtBool = true;
                }
                else if (gamepad2.right_trigger < (0.5 - histInterval / 2.0)){
                    rtBool = false;
                }
                if(gamepad2.left_trigger > (0.5 + histInterval / 2.0)) {
                    ltBool = true;
                }
                else if (gamepad2.left_trigger < (0.5 - histInterval / 2.0)){
                    ltBool = false;
                }

                if(rtBool != rtBoolLast){
                    if(rtBool){
                        if(func.extensorState == ExtensorState.RETRACTED){
                            func.extensorState = ExtensorState.HALF_EXTENDED;
                        }
                        else if(func.extensorState == ExtensorState.HALF_EXTENDED){
                            func.extensorState = ExtensorState.FULL_EXTENDED;
                        }
                    }
                    rtBoolLast = rtBool;
                }
                if(ltBool != ltBoolLast){
                    if(ltBool){
                        if(func.extensorState == ExtensorState.FULL_EXTENDED){
                            func.extensorState = ExtensorState.HALF_EXTENDED;
                        }
                        else if(func.extensorState == ExtensorState.HALF_EXTENDED){
                            func.extensorState = ExtensorState.RETRACTED;
                        }
                    }
                    ltBoolLast = ltBool;
                }
                func.setExtinderePoz();
                func.articulatieGheruta.setPosition(func.pozArticulator);
            }
            //art luat din intake 0.128
            //art pus pe bara 0.185
            //art luat de pe gard 0.429

            //extindere retras: 0.07
            //extindere intermediar: 0.17
            //extindere extins: 0.275
        }
    });
    public void stop() {
        stop = true;
    }
    @Override
    public void loop() {

        telemetry.addData("sliderL:",func.sliderL.getCurrentPosition());
        telemetry.addData("sliderR:",func.sliderR.getCurrentPosition());
        telemetry.addData("setPoint:",pid.getSetpoint());
        telemetry.addData("rtBoolean:", rtBool);
        telemetry.addData("ltBoolean:", ltBool);
        telemetry.addData("Extensor state:", func.extensorState);
        telemetry.addData("blue:",func.colorSensor.blue());
        telemetry.addData("red:",func.colorSensor.red());
        telemetry.addData("green:",func.colorSensor.green());
        telemetry.addData("poz gheruta:",func.gheruta.getPosition());
        telemetry.addData("poz articulatie:",func.articulatieGheruta.getPosition());
        telemetry.addData("poz brat:",func.pozArm);
        telemetry.addData("poz extindere:",func.pozExtindere);
        telemetry.addData("extindere R:", func.extindereR.getPosition());
        telemetry.update();
    }
}

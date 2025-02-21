package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static java.lang.Math.abs;

import static org.firstinspires.ftc.teamcode.Parametri.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Encoder;

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
    boolean initial = false;
    private boolean rtBool, ltBool, rtBoolLast = false, ltBoolLast = false;
    private long extensorExtensionTriggerTime, extensorRetractionTriggerTime;
    private boolean extensorExtensionOverride = false, extensorRetractionOverride = false;
    public double pidResult, ok = 0;
    //public double powIntake = 0.0;
    long lastTime = 0;
    private Encoder right,left,front;
    @Override
    public void init() {
        left = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));
        right = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        front = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));

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

                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x ;
                rx = gamepad1.right_stick_x;

                pmotorFL = y + x + rx;
                pmotorBL = y - x + rx;
                pmotorBR = y + x - rx;
                pmotorFR = y - x - rx;

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
                if(ok == 0){
                    func.targetSliderJos(1,2000);
                    ok = 1;
                }
                else if (gamepad2.right_stick_y != 0.0) {
                    func.setSliderPower(gamepad2.right_stick_y);
                    func.setpointNotActive = true;
                }
                else {
                    if (func.setpointNotActive) {
                        func.setpointNotActive = false;
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
                if(gamepad2.left_stick_y != 0.0){
                    func.setAgatarePower(gamepad2.left_stick_y);
                }
                else{
                    func.setAgatarePower(0);
                }
    
                if(gamepad2.right_bumper){
                    func.pozGherutaSus = 0;
                }
                if(gamepad2.left_bumper){
                    func.pozGherutaSus = 0.3;
                }

                func.setGherutaSusPosition(func.pozGherutaSus);

                func.setArmPosition(func.pozArm);

                if(gamepad2.dpad_up && !apasat && lastTime + 500 < System.currentTimeMillis()){
                    func.ghearatogheara();
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
                    func.pozArm = 0.2;
                    func.pozArticulatorSus = 0.1;
                }
                if(gamepad1.a && initial && lastTime + 500 < System.currentTimeMillis()){
                     func.luat();
                     initial = false;
                     lastTime = System.currentTimeMillis();
                }
                else if (gamepad1.a && !initial && lastTime + 300 < System.currentTimeMillis()) {
                    func.intermediar();
                    initial = true;
                    lastTime = System.currentTimeMillis();
                }
                if(gamepad1.b && initial && lastTime + 500 < System.currentTimeMillis()){
                    func.initiala();
                    initial = false;
                    lastTime = System.currentTimeMillis();
                }
                else if (gamepad1.b && !initial && lastTime + 300 < System.currentTimeMillis()) {
                    func.samples();
                    initial = true;
                    lastTime = System.currentTimeMillis();
                }

                if(gamepad1.left_bumper)
                    func.dreapta();
                else
                    func.mijloc();
                if(gamepad1.right_bumper)
                    func.stanga();

                if(gamepad1.dpad_up){
                    func.gherutaJos.setPosition(0.64);
                }
                if(gamepad1.dpad_down){
                    func.gherutaJos.setPosition(0.855);
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
                if(gamepad2.y) {
                    func.gherutaJos.setPosition(0.855);
                }

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


                /*if(gamepad1.a && func.pozArtClaw < 1){
                    func.pozArtClaw += 0.005;
                }
                if(gamepad1.b && func.pozArtClaw > 0){
                    func.pozArtClaw -= 0.005;
                }*/

                /*if(gamepad1.a && func.pozRotatie < 1){
                    func.pozRotatie += 0.005;
                }
                if(gamepad1.b && func.pozRotatie > 0){
                    func.pozRotatie -= 0.005;
                }*/

                /*if(gamepad1.dpad_left && func.pozRotClaw < 1){
                    func.pozRotClaw += 0.005;
                }
                if(gamepad1.dpad_right && func.pozRotClaw > 0){
                    func.pozRotClaw -= 0.005;
                }*/
                func.articulatieGherutaSus.setPosition(func.pozArticulatorSus);
                func.articulatieGherutaJos.setPosition(func.pozArticulatorJos);
                func.rotatieGherutaJos.setPosition(func.pozRotatieGhearaJos);
                func.rotatiefata.setPosition(func.pozRotatie);
            }

        }
    });
    public void stop() {
        stop = true;
    }
    @Override
    public void loop() {
        telemetry.addData("sliderL:",func.getSliderLPosition());
        telemetry.addData("sliderR:",func.getSliderRPosition());
        telemetry.addData("setPoint:",pid.getSetpoint());
        telemetry.addData("rtBoolean:", rtBool);
        telemetry.addData("ltBoolean:", ltBool);
        telemetry.addData("Extensor state:", func.extensorState);
        telemetry.addData("poz gheruta sus:",func.gherutaSus.getPosition());
        //telemetry.addData("poz articulatie:",func.articulatieGheruta.getPosition());
        telemetry.addData("poz brat:",func.pozArm);
        //telemetry.addData("poz rotatie gheruta :",func.pozRotClaw);
        telemetry.addData("poz brat:",func.pozArm);
        //telemetry.addData("poz articulatie fata:",func.pozArtClaw);
        telemetry.addData("poz rotatie fata:",func.pozRotatie);
        telemetry.addData("poz gheruta fata:",func.pozGherutaJos);
        //telemetry.addData("poz extindere:",func.pozExtindere);
        //telemetry.addData("extindere R:", func.extindereR.getPosition());
        //telemetry.addData("extindere L:", func.extindereL.getPosition());
        telemetry.addData("right:",right.getCurrentPosition());
        telemetry.addData("left:",left.getCurrentPosition());
        telemetry.addData("front:",front.getCurrentPosition());
        telemetry.addData("PID",func.setpointNotActive);
        telemetry.addData("atins",func.atins());
        telemetry.addData("touch L",func.touchL.isPressed());
        telemetry.addData("touch R",func.touchR.isPressed());
        telemetry.addData("rotire gheruta",func.pozRotatieGhearaJos);
        telemetry.update();
    }
}

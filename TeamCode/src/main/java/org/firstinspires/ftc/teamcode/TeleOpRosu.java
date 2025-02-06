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
    GoBildaPinpointDriver odo;
    double sm = 1.0;
    double y, x, rx;
    double max = 0.0;
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public FunctiiDeProgram func = new FunctiiDeProgram();
    boolean stop = false, puneBara = false;
    boolean apasat = false;
    boolean initial = false;
    boolean initial1 = false;
    private boolean rtBool, ltBool, rtBoolLast = false, ltBoolLast = false;
    private long extensorExtensionTriggerTime, extensorRetractionTriggerTime;
    private boolean extensorExtensionOverride = false, extensorRetractionOverride = false;
    public double pidResult;
    //public double powIntake = 0.0;
    long lastTime = 0;
    private Encoder right,left,front;
    @Override
    public void init() {
        left = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));
        right = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        front = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));

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
                    else if(!func.atins()){
                        pidResult = pid.performPID((func.sliderR.getCurrentPosition() + func.sliderL.getCurrentPosition()) / 2.0);
                        func.sliderR.setPower(pidResult);
                        func.sliderL.setPower(pidResult);
                    }
                    else{
                        func.sliderR.setPower(0);
                        func.sliderL.setPower(0);

                    }
                }

                if(gamepad2.right_bumper){
                    func.pozGheruta = 0;
                }
                if(gamepad2.left_bumper){
                    func.pozGheruta = 0.3;
                }
                func.gheruta.setPosition(func.pozGheruta);

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
                    func.pozArm = 0.2              ;
                    func.pozArticulator = 0.1;
                }
                 if(gamepad1.a && !initial && lastTime + 500 < System.currentTimeMillis()){
                     func.samples();
                     initial = true;
                     lastTime = System.currentTimeMillis();
                }
                if(gamepad1.b && !initial1 && lastTime + 500 < System.currentTimeMillis()){
                    func.luat();
                    initial1 = true;
                    lastTime = System.currentTimeMillis();
                }
                else if (gamepad1.b && initial1 && lastTime + 300 < System.currentTimeMillis()) {
                    func.initiala();
                    initial1 = false;
                    lastTime = System.currentTimeMillis();
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

                if(gamepad1.right_trigger > (0.5 + histInterval / 2.0)) {
                    rtBool = true;
                }
                else if (gamepad1.right_trigger < (0.5 - histInterval / 2.0)){
                    rtBool = false;
                }
                if(gamepad1.left_trigger > (0.5 + histInterval / 2.0)) {
                    ltBool = true;
                }
                else if (gamepad1.left_trigger < (0.5 - histInterval / 2.0)){
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
                if(gamepad1.dpad_up){
                    func.deschis();
                }
                if(gamepad1.dpad_down){
                    func.inchis();
                }
                func.setExtinderePoz();
                /*if(gamepad1.a && func.pozArtClaw < 1){
                    func.pozArtClaw += 0.005;
                }
                if(gamepad1.b && func.pozArtClaw > 0){
                    func.pozArtClaw -= 0.005;
                }
                if(gamepad1.dpad_up && func.pozRotatie < 1){
                    func.pozRotatie += 0.005;
                }
                if(gamepad1.dpad_down && func.pozRotatie > 0){
                    func.pozRotatie -= 0.005;
                }

                if(gamepad1.dpad_left && func.pozRotClaw < 1){
                    func.pozRotClaw += 0.005;
                }
                if(gamepad1.dpad_right && func.pozRotClaw > 0){
                    func.pozRotClaw -= 0.005;
                }*/

                func.articulatieGheruta.setPosition(func.pozArticulator);

                func.articulatieClaw2.setPosition(func.pozArtClaw);
                func.rotatieClaw2.setPosition(func.pozRotClaw);
                func.rotatiefata.setPosition(func.pozRotatie);
                func.claw2.setPosition(func.pozGheruta2);

            }

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
        //telemetry.addData("Extensor state:", func.extensorState);
        telemetry.addData("poz gheruta:",func.gheruta.getPosition());
        telemetry.addData("poz articulatie:",func.articulatieGheruta.getPosition());
        telemetry.addData("poz brat:",func.pozArm);
        telemetry.addData("poz rotatie gheruta :",func.pozRotClaw);
        telemetry.addData("poz brat:",func.pozArm);
        telemetry.addData("poz articulatie fata:",func.pozArtClaw);
        telemetry.addData("poz rotatie fata:",func.pozRotatie);
        telemetry.addData("poz gheruta fata:",func.pozGheruta2);
        //telemetry.addData("poz extindere:",func.pozExtindere);
        //telemetry.addData("extindere R:", func.extindereR.getPosition());
        //telemetry.addData("extindere L:", func.extindereL.getPosition());
        telemetry.addData("right:",right.getCurrentPosition());
        telemetry.addData("left:",left.getCurrentPosition());
        telemetry.addData("front:",front.getCurrentPosition());
        telemetry.update();
    }
}

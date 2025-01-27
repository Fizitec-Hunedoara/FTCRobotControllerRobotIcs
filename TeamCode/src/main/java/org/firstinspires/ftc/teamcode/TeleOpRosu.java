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
    boolean stop = false;
    private boolean rtBool, ltBool, rtBoolLast = false, ltBoolLast = false;
    private long extensorExtensionTriggerTime, extensorRetractionTriggerTime;
    private boolean extensorExtensionOverride = false, extensorRetractionOverride = false;
    private final long extensorWaitTime = 200;
    public double pidResult;
    public double powIntake = 0.0;
    @Override
    public void init() {
        func.init(hardwareMap,telemetry,true);
    }
    public void start() {
        //Chassis.start();
        Systems.start();

    }
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while (!stop) {

                //thou fool
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
                if (gamepad2.left_stick_y != 0.0) {
                        func.sliderR.setPower(gamepad2.left_stick_y);
                        func.sliderL.setPower(gamepad2.left_stick_y);
                    func.ceva = true;
                }
                else {
                    if (func.ceva) {
                        func.ceva = false;
                        pid.setSetpoint((func.sliderR.getCurrentPosition() + func.sliderL.getCurrentPosition())/2.0);
                    }
                    else {
                        pidResult = pid.performPID((func.sliderR.getCurrentPosition() + func.sliderL.getCurrentPosition()) / 2.0);
                        func.sliderR.setPower(pidResult);
                        func.sliderL.setPower(pidResult);
                    }
                }
                if(gamepad2.a){
                    powIntake = 0.7;
                }
                else if(gamepad2.b){
                    powIntake = 0;
                }
                func.intake.setPower(powIntake);

            }
        }
    });
    public void stop() {
        stop = true;
    }
    @Override
    public void loop() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        telemetry.addData("sliderL:",func.sliderL.getCurrentPosition());
        telemetry.addData("sliderR:",func.sliderR.getCurrentPosition());
        telemetry.addData("setPoint:",pid.getSetpoint());
        telemetry.addData("pidResult:",pidResult);
        telemetry.addData("rtBoolean:", rtBool);
        telemetry.addData("ltBoolean:", ltBool);
        telemetry.addData("Extensor state:", func.extensorState);
        telemetry.update();
    }
}

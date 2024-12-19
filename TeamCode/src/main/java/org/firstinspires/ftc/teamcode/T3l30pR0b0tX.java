package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Parametri.dslider;
import static org.firstinspires.ftc.teamcode.Parametri.islider;
import static org.firstinspires.ftc.teamcode.Parametri.pslider;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class T3l30pR0b0tX extends OpMode {
    double sm = 1, lb = 1, rb = 1, sliderSlow = 1, intakePos = 0, rotitorPoz = 3, incheieturaPoz = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    boolean stop = false, lastx = false, lasty = false, setSetpoint = true;
    double pozArticulatorGrabber = 0.9,pidResult = 0;
    double ghearaPosition = 0.2;
    long lastTime,lastTimeR,lastTimeL;
    int extins1=0,extins2=0;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(pslider,islider,dslider);
    FunctiiDeProgram func = new FunctiiDeProgram();
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        func.init(hardwareMap,telemetry,true);
    }
    public void start(){
        Chassis.start();
        Systems.start();
        PID.start();
    }
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){

            while(!stop) {

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
                if(gamepad1.x != lastx){
                    rb += 0.5;
                    if(rb > 2){
                        rb = 0.5;
                    }
                }
                if(gamepad1.y != lasty){
                    lb += 0.5;
                    if(lb > 2){
                        lb = 0.5;
                    }
                }
                if(rb == 2){
                    sm = 4;
                }
                else if(lb == 2){
                    sm = 2;
                }
                else{
                    sm = 1;
                }
                lastx = gamepad1.x;
                lasty = gamepad1.y;
                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            }
        }
    });

    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                func.incheieturaBrat.setPower(-gamepad2.right_stick_y);

                if (gamepad2.right_bumper) {
                    func.rotatieGrabber.setPosition(0.7);
                }
                else if (gamepad2.left_bumper) {
                    func.rotatieGrabber.setPosition(0.35);
                }
                else {
                    func.rotatieGrabber.setPosition(0.525);
                }

                if (gamepad2.right_trigger > 0.1) {
                    ghearaPosition = 0.45;
                }
                else if (gamepad2.left_trigger > 0.1) {
                    ghearaPosition = 0.2;
                }
                if (!func.automatizare) {
                    func.gheruta.setPosition(ghearaPosition);
                }
                if (gamepad2.dpad_down) {
                    func.automatizare = true;
                    func.getSpecimen();
                }
                if (gamepad2.dpad_up) {
                    extins1 = 0;
                }
                func.extindere1.setPosition(0.52 - extins1 * 0.32);
                if (gamepad2.a) {
                    extins2 = 0;
                }
                else if (gamepad2.b) {
                    extins2 = 1;
                }
                func.extindere2.setPosition(0.49 + 0.51 * extins2);
                /*if (gamepad2.dpad_up) {
                    func.extindere1.setPosition(0.52);
                } else if (gamepad2.dpad_down) {
                    func.extindere1.setPosition(0.2);
                }*/

                if (gamepad2.dpad_right) {
                    pozArticulatorGrabber = 0.9;
                }
                else if (gamepad2.dpad_left) {
                    pozArticulatorGrabber = 0.1;
                }
                if(!func.automatizare) {
                    func.articulatorGrabber.setPosition(pozArticulatorGrabber);
                }
               /* if (gamepad2.b) {
                    func.extindere2.setPosition(1.0);
                } else if (gamepad2.a) {
                    func.extindere2.setPosition(0.49);
                }*/
            }
        }
    });
    private final Thread PID = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.enable();
            while(!stop){
                pid.setPID(pslider, islider, dslider);

                if (gamepad2.left_stick_y >= 0.1) {
                    func.sliderL.setPower(gamepad2.left_stick_y);
                    func.sliderR.setPower(gamepad2.left_stick_y);
                    setSetpoint = true;
                } else {
                    if (setSetpoint) {
                        setSetpoint = false;
                        pid.setSetpoint(func.sliderR.getCurrentPosition());
                    }
                    if(func.touchR.isPressed() || func.touchL.isPressed()){
                        func.sliderR.setPower(0);
                        func.sliderL.setPower(0);
                    }
                    else {
                        pidResult = pid.performPID(func.sliderR.getCurrentPosition());
                        func.sliderR.setPower(pidResult);
                        func.sliderL.setPower(pidResult);
                    }
                }


            }
        }
    });

    public void stop(){stop = true;}

    @Override
    public void loop() {
        telemetry.addData("i", pid.getI());
        telemetry.addData("p", pid.getP());
        telemetry.addData("d", pid.getD());
        telemetry.addData("error:",pid.getError());
        telemetry.addData("setpoint:",pid.getSetpoint());
        telemetry.addData("senzor L: ",func.touchL.isPressed());
        telemetry.addData("senzor R: ",func.touchR.isPressed());
        telemetry.addData("motorFL:",func.motorFL.getCurrentPosition());
        telemetry.addData("motorBL:",func.motorBL.getCurrentPosition());
        telemetry.addData("motorFR:",func.motorFR.getCurrentPosition());
        telemetry.addData("motorBR:",func.motorBR.getCurrentPosition());
        telemetry.update();
    }
    public void POWER(double df1, double sf1, double ds1, double ss1){
        func.motorFR.setPower(df1);
        func.motorBL.setPower(ss1);
        func.motorFL.setPower(sf1);
        func.motorBR.setPower(ds1);
    }
}
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
public class pdiddy extends OpMode {
    private final double slowSm = 0.4, fastSm = 1;
    private double pmotorFL, pmotorFR, pmotorBL, pmotorBR;
    double sm = 1.0;
    double y, x, rx;
    double max = 0.0;
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public FunctiiDeProgram func = new FunctiiDeProgram();
    boolean stop = false;
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
    public void stop() {
        stop = true;
    }
    @Override
    public void loop() {
        telemetry.addData("sliderL:",func.sliderL.getCurrentPosition());
        telemetry.addData("sliderR:",func.sliderR.getCurrentPosition());
        telemetry.addData("setPoint:",pid.getSetpoint());
        //telemetry.addData("Extensor state:", func.extensorState);
        telemetry.addData("poz gheruta:",func.gheruta.getPosition());
        telemetry.addData("poz articulatie:",func.articulatieGheruta.getPosition());
        telemetry.addData("poz brat:",func.pozArm);
        //telemetry.addData("poz extindere:",func.pozExtindere);
        //telemetry.addData("extindere R:", func.extindereR.getPosition());
        //telemetry.addData("extindere L:", func.extindereL.getPosition());
        //telemetry.addData("intakeR:",func.intakeR.getPosition());
        //telemetry.addData("intakeL:",func.intakeL.getPosition());
        //telemetry.addData("taci:",func.touchL.getValue());
        telemetry.addData("right:",right.getCurrentPosition());
        telemetry.addData("left:",left.getCurrentPosition());
        telemetry.addData("front:",front.getCurrentPosition());
        telemetry.update();
    }
}

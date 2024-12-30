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
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

//@TeleOp
public class motorTest extends OpMode {
    public DcMotorEx sliderL, sliderR, incheieturaBrat;

    double sm = 1, lb = 1, rb = 1, sliderSlow = 1, intakePos = 0, rotitorPoz = 3, incheieturaPoz = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;
    boolean stop = false, lastx = false, lasty = false, intaked = true, gherutaL = false, gherutaR = false, lastBumperL, lastBumperR, setSetpoint = true, automatizare = false, started_left = false, started_right = false, closed_left = false, closed_right = false;
    double servoPos = 0, servoPos2 = 0.85, pidResult = 0;
    double ghearaPosition = 0.2;
    long lastTime, lastTimeR, lastTimeL;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(pslider, islider, dslider);

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);


        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");
        incheieturaBrat = hardwareMap.get(DcMotorEx.class,"MotorMelc");


        sliderL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        incheieturaBrat.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sliderR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        incheieturaBrat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        sliderL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        incheieturaBrat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void start() {
        Chassis.start();
        //Systems.start();
        PID.start();
    }

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                sliderL.setPower(-gamepad2.left_stick_y);
                sliderR.setPower(gamepad2.left_stick_y);

            }
        }
    });



    private final Thread PID = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.enable();
            while (!stop) {
                pid.setPID(pslider, islider, dslider);
                if (gamepad2.left_stick_y != 0) {
                    setSetpoint = true;
                    sliderL.setPower(-gamepad2.left_stick_y);
                    sliderR.setPower(gamepad2.left_stick_y);
                } else if (!automatizare) {
                    if (setSetpoint) {
                        setSetpoint = false;
                        pid.setSetpoint(sliderL.getCurrentPosition());
                    } else {
                        pidResult = pid.performPID(sliderL.getCurrentPosition());
                        sliderL.setPower(-pidResult);
                        sliderR.setPower(pidResult);
                    }
                }
                else {
                    setSetpoint = true;
                }
            }
        }
    });

    public void stop() {
        stop = true;
    }

    @Override
    public void loop() {

        telemetry.addData("sliderL:", sliderL.getPower());
        telemetry.addData("sliderL poz:", sliderL.getCurrentPosition());
        telemetry.addData("sliderR:", sliderR.getPower());
        telemetry.addData("sliderR poz:", sliderR.getCurrentPosition());
        telemetry.addData("joystick: ", gamepad2.left_stick_y);

        telemetry.update();
    }


}

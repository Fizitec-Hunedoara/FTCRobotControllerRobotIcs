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

public class teleopopt extends OpMode {
    public DcMotorEx motorBR, motorBL, motorFL, motorFR, sliderL, sliderR, incheieturaBrat;
    private Servo rotatieGrabber;
    private Servo gheruta;
    private Servo extindere1;
    private Servo articulatorGrabber;
    private Servo extindere2;
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

        motorBL = hardwareMap.get(DcMotorEx.class, "BL");
        motorBR = hardwareMap.get(DcMotorEx.class, "BR");
        motorFL = hardwareMap.get(DcMotorEx.class, "FL");
        motorFR = hardwareMap.get(DcMotorEx.class, "FR");
        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");
        incheieturaBrat = hardwareMap.get(DcMotorEx.class,"MotorMelc");

        rotatieGrabber = hardwareMap.get(Servo.class, "RotG");
        gheruta = hardwareMap.get(Servo.class, "Gheara");
        extindere1 = hardwareMap.get(Servo.class, "Extindere1");
        articulatorGrabber = hardwareMap.get(Servo.class, "ArtG");
        extindere2 = hardwareMap.get(Servo.class, "Extindere2");

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        incheieturaBrat.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sliderL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        incheieturaBrat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        incheieturaBrat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void start() {
        Chassis.start();
        Systems.start();
        PID.start();
    }

    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                y = -gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
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

                if (gamepad1.x != lastx) {
                    rb += 0.5;
                    if (rb > 2) {
                        rb = 0.5;
                    }
                }

                if (gamepad1.y != lasty) {
                    lb += 0.5;
                    if (lb > 2) {
                        lb = 0.5;
                    }
                }

                if (rb == 2) {
                    sm = 4;
                } else if (lb == 2) {
                    sm = 2;
                } else {
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
                incheieturaBrat.setPower(-gamepad2.right_stick_y);

                if (gamepad2.right_bumper) {
                    rotatieGrabber.setPosition(0.7);
                } else if (gamepad2.left_bumper) {
                    rotatieGrabber.setPosition(0.35);
                } else {
                    rotatieGrabber.setPosition(0.525);
                }

                if (gamepad2.right_trigger > 0.1) {
                    ghearaPosition = 0.45;
                } else if (gamepad2.left_trigger > 0.1) {
                    ghearaPosition = 0.2;
                }

                gheruta.setPosition(ghearaPosition);

                if (gamepad2.dpad_up) {
                    extindere1.setPosition(0.52); //cine a facut codul acesta, sa suga pula
                } else if (gamepad2.dpad_down) {
                    extindere1.setPosition(0.2);
                }

                if (gamepad2.dpad_right) {
                    articulatorGrabber.setPosition(0.9);
                } else if (gamepad2.dpad_left) {
                    articulatorGrabber.setPosition(0.1);
                }

                if (gamepad2.b) {
                    extindere2.setPosition(1.0);
                } else if (gamepad2.a) {
                    extindere2.setPosition(0.49);
                }
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
        telemetry.addData("motorFL:", motorFL.getCurrentPosition());
        telemetry.addData("motorBL:", motorBL.getCurrentPosition());
        telemetry.addData("motorBR:", motorBR.getCurrentPosition());
        telemetry.addData("motorFR:", motorFR.getCurrentPosition());
        telemetry.addData("sliderL:", sliderL.getPower());
        telemetry.addData("sliderL poz:", sliderL.getCurrentPosition());
        telemetry.addData("sliderR:", sliderR.getPower());
        telemetry.addData("sliderR poz:", sliderR.getCurrentPosition());
        telemetry.addData("error:", pid.getError());
        telemetry.addData("setpoint:", pid.getSetpoint());
        telemetry.addData("pos gheara:", gheruta.getPosition());
        telemetry.addData("pos articulatie gheara:", articulatorGrabber.getPosition());
        telemetry.addData("pos incheietura brat:", incheieturaBrat.getCurrentPosition());
        telemetry.addData("rotatie gheara:", rotatieGrabber.getPosition());
        telemetry.addData("extindere stanga:", extindere1.getPosition());
        telemetry.addData("extindere dreapta:", extindere2.getPosition());
        telemetry.update();
    }

    public void POWER(double df1, double sf1, double ds1, double ss1) {
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}

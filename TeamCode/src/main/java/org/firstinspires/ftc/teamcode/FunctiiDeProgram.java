package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
public class FunctiiDeProgram {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public DcMotorEx motorBL, motorBR, motorFL, motorFR, sliderL, sliderR, intake;
    public Servo articulatieGheruta, gheruta, extindereR, extindereL, intakeRotatie, intakePliere, armL, armR;
    public boolean automatizare = false, ceva = false, extins = false, initExtins = false;
    public ColorSensor colorSensor;
    //TouchSensor touchL,touchR;
    private boolean sasiuInited;
    private boolean isStopRequested = false;
    public double gherutaPoz = 0.15, sliderTargetPoz = 0, pozArticulatorGrabber = 0.1;
    LinearOpMode opMode;
    public ExtensorState extensorState = ExtensorState.RETRACTED;
    public double pozGheruta = 0.154, pozArticulator = 0.21, pozArm = 0.1,pozExtindere = 0.0;

    public FunctiiDeProgram() {
    }

    public FunctiiDeProgram(LinearOpMode opmode) {
        this.opMode = opmode;
    }

    public void init(HardwareMap hard) {
        this.init(hard, null, false);
    }

    public void init(HardwareMap hard, Telemetry telemetry, boolean shouldInitSasiu) {
        this.hardwareMap = hard;
        this.telemetry = telemetry;
        if (shouldInitSasiu) {
            initSasiu(hard);
        }
        sasiuInited = shouldInitSasiu;

        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        articulatieGheruta = hardwareMap.get(Servo.class, "artG");
        extindereR = hardwareMap.get(Servo.class, "extindereR");
        extindereL = hardwareMap.get(Servo.class, "extindereL");
        intakePliere = hardwareMap.get(Servo.class, "intakeR");
        intakeRotatie = hardwareMap.get(Servo.class, "intakeL");
        armR = hardwareMap.get(Servo.class, "armR");
        armL = hardwareMap.get(Servo.class, "armL");
        gheruta = hardwareMap.get(Servo.class, "claw");

        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
        /*touchL = hardwareMap.get(TouchSensor.class,"touchL");
        touchR = hardwareMap.get(TouchSensor.class,"touchR");*/

        sliderR.setDirection(DcMotorEx.Direction.REVERSE);

        sliderL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sliderR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sliderL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stop() {
        this.isStopRequested = true;
    }

    public void initSasiu(HardwareMap hard) {
        motorBL = hard.get(DcMotorEx.class, "BL"); // Motor Back-Left
        motorBR = hard.get(DcMotorEx.class, "BR"); // Motor Back-Left
        motorFL = hard.get(DcMotorEx.class, "FL"); // Motor Back-Left
        motorFR = hard.get(DcMotorEx.class, "FR"); // Motor Back-Left

        motorBR.setDirection(DcMotorEx.Direction.REVERSE);
        motorFR.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        /*motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);*/

        /*motorFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);*/

        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public synchronized void POWER(double df1, double sf1, double ds1, double ss1) {
        if (sasiuInited) {
            motorFR.setPower(df1);
            motorBL.setPower(ss1);
            motorFL.setPower(sf1);
            motorBR.setPower(ds1);

        }
        else {
            throw new NullPointerException("Bro sasiul nu e initializat");
        }
    }

    public void ansamblul_leleseana(int poz1, int pow, double tolerance) {
        if (poz1 > sliderR.getCurrentPosition()) {
            while (sliderR.getCurrentPosition() < poz1 && !isStopRequested) {
                sliderR.setPower(pow);
                sliderL.setPower(pow);
            }
        }
        else {
            while (sliderR.getCurrentPosition() > poz1 + tolerance && !isStopRequested) {
                sliderR.setPower(-pow);
                sliderL.setPower(-pow);
            }
        }

//            while (slider1.getCurrentPosition() > poz1 || slider1.getCurrentPosition() < poz1 + tolerance){
//                slider2.setVelocity(-vel);
//                slider1.setVelocity(-vel);
//            }
        sliderR.setPower(0);
        sliderL.setPower(0);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
//        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ceva = true;
    }

    public void ansamblul_leleseana_auto(int poz1, int pow, double tolerance) {

        if (poz1 > sliderR.getCurrentPosition()) {
            while (sliderR.getCurrentPosition() < poz1 && this.opMode.opModeIsActive()) {
                sliderR.setPower(pow);
                sliderL.setPower(pow);
            }

        }
        else {
            while (sliderR.getCurrentPosition() > poz1 + tolerance && this.opMode.opModeIsActive()) {
                sliderR.setVelocity(-pow);
                sliderR.setVelocity(-pow);
            }
        }

//            while (slider1.getCurrentPosition() > poz1 || slider1.getCurrentPosition() < poz1 + tolerance){
//                slider2.setVelocity(-vel);
//                slider1.setVelocity(-vel);
//            }
        sliderR.setPower(0);
        sliderL.setPower(0);
        kdf_auto(100);
        //SpateStanga.setPosition(poz_servo_st);
        //SpateDreapta.setPosition(poz_servo_dr);
//        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ceva = true;
    }

    public synchronized void targetSlider(double poz, double pow, double t, int tolerance) {
        automatizare = true;
        if (sliderR.getCurrentPosition() < poz) {
            sliderR.setPower(-pow);
            sliderL.setPower(-pow);
        }
        else {
            sliderL.setPower(pow);
            sliderR.setPower(pow);
        }
        double lastTime = System.currentTimeMillis();
        while (!isStopRequested
                && lastTime + t > System.currentTimeMillis()
                && (abs(sliderR.getCurrentPosition() - poz) > tolerance)) {
        }
        sliderR.setPower(0);
        sliderL.setPower(0);
        automatizare = false;
        ceva = true;
    }

    public synchronized void targetSlider_auto(double poz, double pow, double t, int tolerance) {
        automatizare = true;
        double lastTime = System.currentTimeMillis();
        if (sliderR.getCurrentPosition() < poz) {
            while (sliderR.getCurrentPosition() < poz - tolerance && opMode.opModeIsActive() && lastTime + t > System.currentTimeMillis()) {
                sliderR.setPower(-pow);
                sliderL.setPower(-pow);
            }
        }
        else {
            while (sliderR.getCurrentPosition() > poz + tolerance && opMode.opModeIsActive() && lastTime + t > System.currentTimeMillis()) {
                sliderL.setPower(pow);
                sliderR.setPower(pow);
            }
        }
        sliderR.setPower(0);
        sliderL.setPower(0);
        sliderTargetPoz = poz;
        automatizare = false;
    }

    public synchronized void targetSliderJos(double pow, double t) {
        automatizare = true;

        sliderL.setPower(pow);
        sliderR.setPower(pow);

        double lastTime = System.currentTimeMillis();
        sliderR.setPower(0);
        sliderL.setPower(0);
        //ceva = true;
    }

    public synchronized void targetSliderJos_auto(double pow, double t) {
        automatizare = true;
        double lastTime = System.currentTimeMillis();
        sliderR.setPower(0);
        sliderL.setPower(0);
        sliderTargetPoz = 0;
        automatizare = false;
        //ceva = true;
    }

    public synchronized void target(double poz, double vel, DcMotorEx motor, double t, int tolerance) {
        double lastTime = System.currentTimeMillis();
        if (motor.getCurrentPosition() < poz) {
            while (!isStopRequested && motor.getCurrentPosition() < poz - tolerance && lastTime + t > System.currentTimeMillis()) {
                motor.setVelocity(vel);
            }
        }
        else {
            while (!isStopRequested && motor.getCurrentPosition() > poz + tolerance && lastTime + t > System.currentTimeMillis()) {
                motor.setVelocity(-vel);
            }
        }
        motor.setVelocity(0);
        ceva = true;
    }

    public synchronized void target_auto(double poz, double vel, DcMotorEx motor, double t, int tolerance) {
        double lastTime = System.currentTimeMillis();
        if (motor.getCurrentPosition() < poz) {
            /*if(hardwareMap.voltageSensor.iterator().next().getVoltage() < 13){
                poz = poz * 0.9;
            }*/
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < poz - tolerance && lastTime + t > System.currentTimeMillis()) {
                motor.setVelocity(vel);
            }
        }
        else {
            /*if(hardwareMap.voltageSensor.iterator().next().getVoltage() < 13){
                poz = poz * 1.1;
            }*/
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > poz + tolerance && lastTime + t > System.currentTimeMillis()) {
                motor.setVelocity(-vel);
            }
        }
        motor.setVelocity(0);
    }

    public double getBatteryVoltage() {
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void deschidere() {
        pozGheruta = 0.154;
    }
    public void inchidere() {
        pozGheruta = 0.0;
    }
    public void intaketogheara(){
        Thread t1 = new Thread(() -> {
            inchidere();
            kdf(500);
            pozArm = 0.2;
        });
        t1.start();
    }
    public void ghearatocos(){
        pozArm = 0.51;
        pozArticulator = 0.3;
    }
    public void gardtogheara(){
        Thread t1 = new Thread(() -> {
            pozArticulator = 0;
            kdf(200);
            pozArm = 0.2;
            kdf(200);
            pozArticulator = 0.5;
            pozArm = 0.01;
            kdf(200);
            inchidere();
        });
        t1.start();
    }
    public void ghearapozbara(){
        Thread t1 = new Thread(() -> {
            pozArticulator = 0.8;
            pozArm = 0.55;
            kdf(200);
            pozArticulator = 0.5;
        });
        t1.start();
    }
    public void puspebara(){
        Thread t1 = new Thread(() -> {
            pozArticulator = 0.1;
            pozArm = 0.11;
            kdf(200);
            deschidere();
        });
        t1.start();
    }
    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis() && !isStopRequested) ;
    }

    public void kdf_auto(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis() && opMode.opModeIsActive()) ;
    }

    public void setExtinderePoz() {
        switch (extensorState) {
            case RETRACTED:
                extindereL.setPosition(0.07);
                extindereR.setPosition(0.93);
                break;
            case HALF_EXTENDED:
                extindereL.setPosition(0.17);
                extindereR.setPosition(0.83);
                break;
            case FULL_EXTENDED:
                extindereL.setPosition(0.275);
                extindereR.setPosition(0.725);
                break;
        }
    }
   /* public boolean atins(){
        return touchL.isPressed() || touchR.isPressed();
    }*/
}
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class FunctiiDeProgram{
    TouchSensor touchL,touchR;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public DcMotorEx motorBL, motorBR, motorFL, motorFR, sliderL, sliderR, incheieturaBrat;
    public Servo rotatieGrabber,gheruta, extindere1,articulatorGrabber,extindere2;
    public boolean automatizare = false, ceva = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private CRServo maceta, extensorL, extensorR;
    private AnalogInput potentiometru;
    private DistanceSensor distanceL, distanceR;
    private boolean sasiuInited;
    private boolean isStopRequested = false;
    public double gherutaPoz = 0.15;

    public void init(HardwareMap hard){
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
        incheieturaBrat = hardwareMap.get(DcMotorEx.class,"MotorMelc");

        rotatieGrabber = hardwareMap.get(Servo.class, "RotG");
        gheruta = hardwareMap.get(Servo.class, "Gheara");
        extindere1 = hardwareMap.get(Servo.class, "Extindere1");
        articulatorGrabber = hardwareMap.get(Servo.class, "ArtG");
        extindere2 = hardwareMap.get(Servo.class, "Extindere2");

        touchL = hardwareMap.get(TouchSensor.class,"touchL");
        touchR = hardwareMap.get(TouchSensor.class,"touchR");

        sliderR.setDirection(DcMotorEx.Direction.REVERSE);

        sliderL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        incheieturaBrat.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sliderR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        incheieturaBrat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        sliderL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        incheieturaBrat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void stop(){
        this.isStopRequested = true;
    }

    public void initSasiu(HardwareMap hard) {
        motorBL = hard.get(DcMotorEx.class, "BL"); // Motor Back-Left
        motorBR = hard.get(DcMotorEx.class, "BR"); // Motor Back-Left
        motorFL = hard.get(DcMotorEx.class, "FL"); // Motor Back-Left
        motorFR = hard.get(DcMotorEx.class, "FR"); // Motor Back-Left

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        /*motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);*/
        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
    public void ansamblul_leleseana(int poz1,int pow,double tolerance){

        if (poz1 > sliderR.getCurrentPosition()){
            while (sliderR.getCurrentPosition() < poz1 && !isStopRequested){
                sliderR.setPower(pow);
                sliderL.setPower(pow);
            }

        }
        else {
            while (sliderR.getCurrentPosition()>poz1 + tolerance && !isStopRequested ){
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

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        //SpateStanga.setPosition(poz_servo_st);
        //SpateDreapta.setPosition(poz_servo_dr);
//        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ceva = true;
    }

    public  void pule_lule(int poz1,int vel,double tolerance){

        if (poz1 > incheieturaBrat.getCurrentPosition()){
            while (incheieturaBrat.getCurrentPosition() < poz1 && !isStopRequested){
                incheieturaBrat.setVelocity(vel);
            }

        }
        else {
            while (incheieturaBrat.getCurrentPosition()>poz1 + tolerance && !isStopRequested ){
                incheieturaBrat.setVelocity(-vel);
            }
        }

//            while (slider1.getCurrentPosition() > poz1 || slider1.getCurrentPosition() < poz1 + tolerance){
//                slider2.setVelocity(-vel);
//                slider1.setVelocity(-vel);
//            }
        incheieturaBrat.setVelocity(0);


        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        //SpateStanga.setPosition(poz_servo_st);
        //SpateDreapta.setPosition(poz_servo_dr);
        incheieturaBrat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
        ceva = true;
    }
    public synchronized void targetSliderJos(double poz, double pow, double t, int tolerance) {
        automatizare = true;

        sliderL.setPower(pow);
        sliderR.setPower(pow);

        double lastTime = System.currentTimeMillis();
        while (!isStopRequested
                && lastTime + t > System.currentTimeMillis()
                && !(touchL.isPressed() || touchR.isPressed())) {
        }
        sliderR.setPower(0);
        sliderL.setPower(0);
        //ceva = true;
    }
    public synchronized void target(double poz, double vel, DcMotorEx motor, double t, int tolerance) {
        if (motor.getCurrentPosition() < poz) {
            motor.setVelocity(vel);
        }
        else {
            motor.setVelocity(-vel);
        }
        double lastTime = System.currentTimeMillis();
        while (!isStopRequested
                && lastTime + t > System.currentTimeMillis()
                && (abs(motor.getCurrentPosition() - poz) > tolerance)) {
        }
        motor.setVelocity(0);
        ceva = true;
    }
    public void deschidere(){
        //gheruta.setPosition(0.45);
        gherutaPoz = 0.45;
    }
    public void inchidere(){
        gheruta.setPosition(0.15);
    }
    public void getSpecimen(){
        Thread t1 = new Thread(() -> {
            if (incheieturaBrat.getCurrentPosition() < 500){
                //target(570,3000,incheieturaBrat,5000,10);
                pule_lule(570,3000,10);
            }
            else
                pule_lule(590,2000,10);
            articulatorGrabber.setPosition(0.65);
        });
        t1.start();
        if (sliderR.getCurrentPosition() > 100)
         targetSliderJos(0,0.5,5000,10);



        deschidere();
    }
    public void chill(){
        Thread t2 = new Thread(() -> {
            pule_lule(580,2000,10);
            articulatorGrabber.setPosition(0.45);
        });
        t2.start();
        if (sliderR.getCurrentPosition()>100)
            targetSliderJos(0,0.5,5000,10);



    }
    public void putSpecimenOnBar(){

        //target(1350,5000,incheieturaBrat,5000,10);
        Thread t3 = new Thread(() -> {
            pule_lule(1420,2000,10);

        });
        t3.start();
        gherutaPoz = 0.15;
        articulatorGrabber.setPosition(0.85);
        //targetSlider(600,1,5000,10);
        //ansamblul_leleseana(450,-1,10);

    }
    public void skibidi_dop_dop_dop(){
        Thread t4 = new Thread(() -> {
            pule_lule(1420,3000,10);
        });
        t4.start();
        articulatorGrabber.setPosition(0.7);
        ansamblul_leleseana(1000,-1,5);



    }
    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis());
    }
}
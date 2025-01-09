package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class FunctiiDeProgram{
    TouchSensor touchL,touchR;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public DcMotorEx motorBL, motorBR, motorFL, motorFR, sliderL, sliderR, incheieturaBrat;
    public Servo rotatieGrabber,gheruta, extindere1,articulatorGrabber,extindere2;
    public boolean automatizare = false, ceva = false, extins = false, initExtins = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private CRServo maceta;
    private AnalogInput potentiometru;
    private DistanceSensor distanceL, distanceR;
    private boolean sasiuInited;
    private boolean isStopRequested = false;
    public double gherutaPoz = 0.15,sliderTargetPoz = 0,pozArticulatorGrabber = 0.1;
    LinearOpMode opMode;
    public ExtensorState extensorState = ExtensorState.RETRACTED;

    public FunctiiDeProgram(){}
    public FunctiiDeProgram(LinearOpMode opmode){
        this.opMode = opmode;
    }
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
        if(!shouldInitSasiu) {
            sliderR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            incheieturaBrat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
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
    public void ansamblul_leleseana(int poz1,int pow,double tolerance){
        if (poz1 > sliderR.getCurrentPosition()){
            while (sliderR.getCurrentPosition() < poz1 && !isStopRequested){
                sliderR.setPower(pow);
                sliderL.setPower(pow);
            }
        }
        else {
            while (sliderR.getCurrentPosition()>poz1 + tolerance && !isStopRequested){
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
        //SpateStanga.setPosition(poz_servo_st);
        //SpateDreapta.setPosition(poz_servo_dr);
//        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ceva = true;
    }
    public void ansamblul_leleseana_auto(int poz1,int pow,double tolerance){

        if (poz1 > sliderR.getCurrentPosition()){
            while (sliderR.getCurrentPosition() < poz1 && this.opMode.opModeIsActive()){
                sliderR.setPower(pow);
                sliderL.setPower(pow);
            }

        }
        else {
            while (sliderR.getCurrentPosition()>poz1 + tolerance && this.opMode.opModeIsActive()){
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
    public  void pule_lule_auto(int poz1,int vel,double tolerance){

        if (poz1 > incheieturaBrat.getCurrentPosition()){
            while (incheieturaBrat.getCurrentPosition() < poz1 && this.opMode.opModeIsActive()){
                incheieturaBrat.setVelocity(vel);
            }

        }
        else {
            while (incheieturaBrat.getCurrentPosition()>poz1 + tolerance && this.opMode.opModeIsActive() ){
                incheieturaBrat.setVelocity(-vel);
            }
        }

//            while (slider1.getCurrentPosition() > poz1 || slider1.getCurrentPosition() < poz1 + tolerance){
//                slider2.setVelocity(-vel);
//                slider1.setVelocity(-vel);
//            }
        incheieturaBrat.setVelocity(0);
        kdf_auto(100);
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
        automatizare = false;
        ceva = true;
    }
    public synchronized void targetSlider_auto(double poz, double pow, double t, int tolerance) {
        automatizare = true;
        double lastTime = System.currentTimeMillis();
        if (sliderR.getCurrentPosition() < poz) {
            while(sliderR.getCurrentPosition() < poz - tolerance && opMode.opModeIsActive() && lastTime + t > System.currentTimeMillis()) {
                sliderR.setPower(-pow);
                sliderL.setPower(-pow);
            }
        }
        else {
            while(sliderR.getCurrentPosition() > poz + tolerance && opMode.opModeIsActive() && lastTime + t > System.currentTimeMillis()) {
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
        while (!isStopRequested
                && lastTime + t > System.currentTimeMillis()
                && !(touchL.isPressed() || touchR.isPressed())) {
        }
        sliderR.setPower(0);
        sliderL.setPower(0);
        //ceva = true;
    }
    public synchronized void targetSliderJos_auto(double pow, double t) {
        automatizare = true;
        double lastTime = System.currentTimeMillis();
        while(!touchR.isPressed() && !touchL.isPressed() && lastTime + t > System.currentTimeMillis() && opMode.opModeIsActive()) {
            sliderL.setPower(pow);
            sliderR.setPower(pow);
        }
        sliderR.setPower(0);
        sliderL.setPower(0);
        sliderTargetPoz = 0;
        automatizare = false;
        //ceva = true;
    }
    public synchronized void target(double poz, double vel, DcMotorEx motor, double t, int tolerance) {
        double lastTime = System.currentTimeMillis();
        if (motor.getCurrentPosition() < poz) {
            while(!isStopRequested && motor.getCurrentPosition() < poz - tolerance && lastTime + t > System.currentTimeMillis()) {
                motor.setVelocity(vel);
            }
        }
        else {
            while(!isStopRequested && motor.getCurrentPosition() > poz + tolerance && lastTime + t > System.currentTimeMillis()) {
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
            while(opMode.opModeIsActive() && motor.getCurrentPosition() < poz - tolerance && lastTime + t > System.currentTimeMillis()) {
                motor.setVelocity(vel);
            }
        }
        else {
            /*if(hardwareMap.voltageSensor.iterator().next().getVoltage() < 13){
                poz = poz * 1.1;
            }*/
            while(opMode.opModeIsActive() && motor.getCurrentPosition() > poz + tolerance && lastTime + t > System.currentTimeMillis()) {
                motor.setVelocity(-vel);
            }
        }
        motor.setVelocity(0);
    }
    public double getBatteryVoltage(){
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }
    public void deschidere(){
        //gheruta.setPosition(0.45);
        gherutaPoz = 0.45;
    }
    public void inchidere(){
        gherutaPoz = 0.15;
    }
    public void getSpecimen(){
        pozArticulatorGrabber = 0.6;
        articulatorGrabber.setPosition(pozArticulatorGrabber);
        Thread t1 = new Thread(() -> {
            if (incheieturaBrat.getCurrentPosition() < 500){
                target(-1270,3000,incheieturaBrat,5000,10);
                //pule_lule(570,3000,10);
            }
            else
                target(-1270,3000,incheieturaBrat,5000,10);
        });
        t1.start();
        Thread t2 = new Thread(() -> {
            if (sliderR.getCurrentPosition() > 100)
                targetSliderJos(1,5000);
        });
        t2.start();
        deschidere();
    }
    public void getSpecimen_auto(){
        rotatieGrabber.setPosition(0.525);
        targetSlider_auto(0,1,5000,10);
        target_auto(-1250,3000,incheieturaBrat,5000,10);
        pozArticulatorGrabber = 0.6;
        deschidere();
    }
    public void chill(){
        pozArticulatorGrabber =0.45;
        articulatorGrabber.setPosition(0.45);
        Thread t1 = new Thread(() -> {
            target(580,2000,incheieturaBrat,5000,10);

        });
        t1.start();
        Thread t2 = new Thread (() -> {
            if (sliderR.getCurrentPosition()>100)
                targetSliderJos(1,5000);
        });
        t2.start();
    }
    public void chill_auto(){
        Thread t2 = new Thread(() -> {
            target_auto(580,5000,incheieturaBrat,5000,10);
            articulatorGrabber.setPosition(0.45);
        });
        t2.start();
        if (sliderR.getCurrentPosition()>100)
            targetSliderJos_auto(1,5000);
    }
    public void putSpecimenOnBar(){

        //target(1350,5000,incheieturaBrat,5000,10);
        pozArticulatorGrabber =0.75;
        articulatorGrabber.setPosition(0.75);
        Thread t3 = new Thread(() -> {
            target(900,2000,incheieturaBrat,5000,10);
        });
        t3.start();
        /*Thread t4 = new Thread(() -> {
            targetSlider(600,1,5000,10);
        });
        t4.start();
        kdf(2000);*/
    }
    public void putSpecimenOnBar_auto(){

        //target(1350,5000,incheieturaBrat,5000,10);
        if(opMode.opModeIsActive()) {
            gherutaPoz = 0.15;
            pozArticulatorGrabber = 0.85;
            target_auto(850, 2000, incheieturaBrat,5000,10);
            targetSlider_auto(750,1,5000,10);
            deschidere();
        }
        //ansamblul_leleseana(450,-1,10);
    }
    public void setArticulatorPoz(double poz){
        /*articulatorGrabber.setPosition(0);
        kdf_auto(10);*/
        articulatorGrabber.setPosition(poz);
    }
    public void pus_in_cos(){
        pozArticulatorGrabber =1;
        Thread t1 = new Thread(() -> {
            target(1200,5000,incheieturaBrat,5000,10);
        });
        t1.start();
        Thread t2 = new Thread(() -> {
            targetSlider(1500,1,5000,10);
        });
        t2.start();
    }
    public void pus_in_cos_auto(){
        target_auto(1100,5000,incheieturaBrat,5000,10);
        targetSlider_auto(1500,1,5000,10);
        pozArticulatorGrabber =1;
        kdf_auto(500);
        deschidere();
        kdf_auto(500);
        pozArticulatorGrabber = 0.5;
        extins = false;
        sliderTargetPoz = 0;
        target_auto(0,5000,incheieturaBrat,5000,10);
    }
    public void ia_de_jos(){
        articulatorGrabber.setPosition(0.2);
        pozArticulatorGrabber =0.2;
        Thread t1 = new Thread(() -> {
            target(-1400, 5000, incheieturaBrat, 5000, 10);
        });
        t1.start();
        if(sliderR.getCurrentPosition() > 100) {
            Thread t2 = new Thread(() -> {
                targetSlider(0, 1, 5000, 10);
            });
            t2.start();
        }
    }
    public void ia_de_jos_auto(){
        pozArticulatorGrabber = 0.25;
        extins = true;
        target_auto(-1600,5000,incheieturaBrat,5000,10);
        kdf_auto(200);
        inchidere();
        kdf_auto(200);
        target_auto(0,5000,incheieturaBrat,5000,10);
    }
    public void kdf(long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis() && !isStopRequested);
    }
    public void kdf_auto( long t) {
        long lastTime = System.currentTimeMillis();
        while (lastTime + t > System.currentTimeMillis() && opMode.opModeIsActive());
    }
    public void doExtensor() {
        switch (extensorState){
            case RETRACTED:
                extindere1.setPosition(0.52);
                extindere2.setPosition(0.49);
                break;
            case HALF_EXTENDED:
                extindere1.setPosition(0.2);
                extindere2.setPosition(0.49);
                break;
            case FULL_EXTENDED:
                extindere1.setPosition(0.2);
                extindere2.setPosition(1);
                break;
        }
    }
}
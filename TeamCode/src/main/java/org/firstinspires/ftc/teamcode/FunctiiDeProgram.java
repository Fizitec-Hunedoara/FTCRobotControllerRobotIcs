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
    public Servo articulatieGheruta, gheruta, extindereR, extindereL, intakeR, intakeL, armL, armR, putaluiLuca;
    public boolean automatizare = false, ceva = false, extins = false, initExtins = false;
    public ColorSensor colorSensor;
    TouchSensor touchL,touchR;
    private boolean sasiuInited;
    private boolean isStopRequested = false;
    public double sliderTargetPoz = 0;
    LinearOpMode opMode;
    public ExtensorState extensorState = ExtensorState.RETRACTED;
    public double pozGheruta = 0, pozArticulator = 0.18, pozArm = 0.1,pozExtindere = 0.0, pozRotatie = 0.0, pozPliere = 0.0, pozPuta = 0.85;

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
        intakeL = hardwareMap.get(Servo.class, "intakeL");
        intakeR = hardwareMap.get(Servo.class, "intakeR");
        armR = hardwareMap.get(Servo.class, "armR");
        armL = hardwareMap.get(Servo.class, "armL");
        gheruta = hardwareMap.get(Servo.class, "claw");
        putaluiLuca = hardwareMap.get(Servo.class, "putaluiLuca");

        //colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
        touchL = hardwareMap.get(TouchSensor.class,"touchL");
        touchR = hardwareMap.get(TouchSensor.class,"touchR");

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

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

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
        pozGheruta = 0.3;
    }
    public void inchidere() {
        pozGheruta = 0;
    }

    public void intaketogheara(){
        Thread t1 = new Thread(() -> {
            pozArm = 0.04;
            pozArticulator = 0.22;
            kdf(200);
            inchidere();
        });
        t1.start();
    }
    public void ghearatocos(){
        pozArm = 0.04;
        kdf(300);
        pozArm = 0.52;
        pozArticulator = 0.4;
        //kdf(300);
        //pozArm = 0.52;
        //pozArticulator = 0.575;
    }
    public void gardtogheara(){
        Thread t1 = new Thread(() -> {
            pozArticulator = 0.465;
            kdf(200);
            pozArm = 0.07;
            kdf(200);
        });
        t1.start();
    }
    public void gardtogheara_auto(){
        pozArticulator = 0.465;
        kdf_auto(200);
        pozArm = 0.07;
        kdf_auto(200);
    }
    public void ghearapozbara(){
        Thread t1 = new Thread(() -> {
            inchidere();
            kdf(200);
            pozArticulator = 0.7054;
            pozArm = 0.49;
            kdf(500);
            pozGheruta = 0.13;
            kdf(200);
            inchidere();
        });
        t1.start();
    }
    public void ghearapozbara_auto(boolean release){
        inchidere();
        kdf_auto(200);
        pozArticulator = 0.7054;
        pozArm = 0.49;
        kdf_auto(500);
        if(release) {
            pozGheruta = 0.13;
            kdf_auto(200);
        }
        inchidere();
    }

    public void puspebara(){
        Thread t1 = new Thread(() -> {
            pozArticulator = 0.6;
            pozArm = 0.5;
            kdf(200);
            pozArticulator = 0.3;
            pozArm = 0.3;
            kdf(500);
            deschidere();
        });
        t1.start();
    }
    public void puspebara_auto(){
        pozArticulator = 0.6;
        pozArm = 0.5;
        kdf_auto(200);
        pozArticulator = 0.3;
        pozArm = 0.3;
        kdf_auto(500);
        deschidere();
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
                extindereL.setPosition(0.01);
                extindereR.setPosition(0.99);
                intakeR.setPosition(0.625);
                intakeL.setPosition(0.79);
                break;
            case OUT:
                extindereL.setPosition(0.01);
                extindereR.setPosition(0.99);
                intakeR.setPosition(0.11);
                intakeL.setPosition(0.09);
                break;
            case HALF_EXTENDED:
                extindereL.setPosition(0.17);
                extindereR.setPosition(0.75);
                intakeR.setPosition(0.11);
                intakeL.setPosition(0.09);
                break;
            case FULL_EXTENDED:
                extindereL.setPosition(0.275);
                extindereR.setPosition(0.73);
                intakeR.setPosition(0.11);
                intakeL.setPosition(0.09);
                break;
        }
    }
   public boolean atins(){
        return touchL.isPressed() || touchR.isPressed();
    }
}
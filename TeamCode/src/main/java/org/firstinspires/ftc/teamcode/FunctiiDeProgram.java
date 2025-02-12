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
    public DcMotorEx motorBL, motorBR, motorFL, motorFR, sliderL, sliderR;
    public Servo articulatieGheruta, gheruta, extindereR, extindereL, armL, armR,  rotatieClaw2, claw2, articulatieClaw2, rotatiefata;
    public boolean automatizare = false, ceva = false, extins = false, initExtins = false;
    public ColorSensor colorSensor;
    TouchSensor touchL,touchR;
    private boolean sasiuInited;
    private boolean isStopRequested = false;
    public double sliderTargetPoz = 0;
    LinearOpMode opMode;
    public ExtensorState extensorState = ExtensorState.RETRACTED;
    public double pozGheruta = 0.3, pozArticulator = 0.1, pozArm = 0.3, pozExtindere = 0.0, pozRotatie = 0, pozGheruta2 = 0.855, pozArtClaw = 0.0, pozRotClaw = 0.185;
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

        articulatieGheruta = hardwareMap.get(Servo.class, "artG");
        extindereR = hardwareMap.get(Servo.class, "extindereR");
        extindereL = hardwareMap.get(Servo.class, "extindereL");
        armR = hardwareMap.get(Servo.class, "armR");
        armL = hardwareMap.get(Servo.class, "armL");
        gheruta = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        rotatieClaw2 = hardwareMap.get(Servo.class, "rotatieClaw2");
        rotatiefata = hardwareMap.get(Servo.class, "rotatiefata");
        articulatieClaw2 = hardwareMap.get(Servo.class, "articulatieClaw2");

        touchL = hardwareMap.get(TouchSensor.class,"touchL");
        touchR = hardwareMap.get(TouchSensor.class,"touchR");

        sliderR.setDirection(DcMotorEx.Direction.REVERSE);

        sliderL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sliderR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sliderL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
    public synchronized void setSliderPower(double power){
        sliderR.setPower(power);
        sliderL.setPower(power);
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
    //public void deschis(){ pozGheruta2 = 0.64; }
    //public void inchis(){ pozGheruta2 = 0.855; }
    public void stanga(){
        pozRotClaw = 0;
    }
    public void dreapta(){
        pozRotClaw = 0.35;
    }
    public void mijloc(){
        pozRotClaw = 0.185;
    }

    public void ghearatogheara(){
        Thread t1 = new Thread(() -> {
            pozArm = 0.13;
            pozArticulator = 0;
            kdf(200);
            inchidere();
            kdf(248);
            claw2.setPosition(0.855);
        });
        t1.start();
    }
    public void ghearatocos(){
        pozArm = 0.5;
        pozArticulator = 0.56;

    }
    public void gardtogheara(){
        Thread t1 = new Thread(() -> {
            pozArticulator = 0.465;
            kdf(200);
            pozArm = 0.07;
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
            pozArm = 0.52;
            kdf(200);
            pozArticulator = 0.3;
            pozArm = 0.3;
            /*kdf(500);
            deschidere();*/
        });
        t1.start();
    }
    public void puspebaraintermediar(){
        Thread t1 = new Thread(() -> {
            pozArticulator = 0.3;
            pozArm = 0.3;
            kdf(500);
            deschidere();
        });
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

    public void initiala(){
        pozRotClaw = 0.18;
        pozArtClaw = 0.03;
        pozRotatie = 0;
    }
    public void samples(){
        pozRotClaw = 0.18;
        pozArtClaw = 0.1;
        pozRotatie = 0.685;
    }
    public void luat(){
        Thread t1 = new Thread(() -> {
            claw2.setPosition(0.855);
            pozRotClaw = 0.18;
            pozArtClaw = 0.05;
            pozRotatie = 0.77;
            kdf(500);
            claw2.setPosition(0.64);
        });
        t1.start();
    }
    public void intermediar(){
        claw2.setPosition(0.64);
        pozRotClaw = 0.18;
        pozArtClaw = 0.05;
        pozRotatie = 0.45;
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
                break;
            case HALF_EXTENDED:
                extindereL.setPosition(0.17);
                extindereR.setPosition(0.85);
                break;
            case FULL_EXTENDED:
                extindereL.setPosition(0.275);
                extindereR.setPosition(0.79);
                break;
        }
    }
   public boolean atins(){
        return touchL.isPressed() || touchR.isPressed();
    }
}
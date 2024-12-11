/* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
*/
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

@TeleOp
public class T3l30pR0b0tX extends OpMode {
    public DcMotorEx motorBR,motorBL,motorFL,motorFR, sliderL, sliderR, incheieturaBrat;
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
    boolean stop = false, lastx = false, lasty = false, intaked = true, gherutaL = false, gherutaR = false, lastBumperL, lastBumperR, setSetpoint = true, automatizare = false, started_left = false, started_right = false,closed_left = false, closed_right = false;
    double servoPos = 0, servoPos2 = 0.85, pidResult = 0;
    double ghearaPosition = 0.2;
    long lastTime,lastTimeR,lastTimeL;
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(pslider,islider,dslider);
    //FunctiiBuneSaSperam c = new FunctiiBuneSaSperam();
    /*Functia de init se ruleaza numai o data, se folosete pentru initializarea motoarelor si chestii :)*/
    @Override
    public void init() {
        //c.initSisteme(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/
        motorBL = hardwareMap.get(DcMotorEx.class, "BL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "BR"); // Motor Back-Left
        motorFL = hardwareMap.get(DcMotorEx.class, "FL"); // Motor Back-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "FR"); // Motor Back-Left
        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");
        incheieturaBrat = hardwareMap.get(DcMotorEx.class,"MotorMelc");

        rotatieGrabber = hardwareMap.get(Servo.class, "RotG");
        gheruta = hardwareMap.get(Servo.class, "Gheara");
        extindere1 = hardwareMap.get(Servo.class, "Extindere1");
        articulatorGrabber = hardwareMap.get(Servo.class, "ArtG");
        extindere2 = hardwareMap.get(Servo.class, "Extindere2");
        /*Liniile astea de cod fac ca motoarele sa aiba puterea inversata fata de cum erau initial,
        sunt fol++osite pentru a face robotul sa mearga in fata dand putere pozitiva la toate cele 4 motoare. */
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);

        /*Liniile astea de cod fac ca motoarele sa poata frana de tot atunci cand ii dai sa franeze*/
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        incheieturaBrat.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sliderL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        incheieturaBrat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        /*Liniile astea de cod fac ca robotul sa mearga cu ajutorul encoderelor(maresc precizia)*/
        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //sliderL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        incheieturaBrat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void start(){
        //c.deschidere();
        Chassis.start();
        Systems.start();
        PID.start();
    }
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while(!stop) {
                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y  = -gamepad1.left_stick_y;
                x  = gamepad1.left_stick_x ;
                rx = gamepad1.right_stick_x;

                /* Liniile astea de cod iau niste variabile care reprezinta puterea fiecarui motor, cu ajutorul puterilor de la controller*/
                pmotorFL = y + x + rx;
                pmotorBL = y - x + rx;
                pmotorBR = y + x - rx;
                pmotorFR = y - x - rx;

                /*Secventele urmatoare de cod stabilesc maximul dintre modulele puterilor motoarelor cu un anumit scop...*/
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
                /*...care este punerea tuturor puterilor motoarelor sub 1, cum puterile de la motoare pot fi numai intre 1 si -1*/
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
    /*Aici se declara thread-ul cu numele systems, pentru ca contine partea de program care se ocupa de sisteme*/
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
                incheieturaBrat.setPower(-gamepad2.right_stick_y);
//                if(gamepad2.left_trigger > 0){
//                    c.pixelIncheietura();
//                }
//                if(gamepad2.right_stick_y > 0){
//                    c.tablaIncheietura();
//                }
                if (gamepad2.right_bumper) {
                    rotatieGrabber.setPosition(0.7);  // Setează o poziție când este apăsat right_bumper
                } else if (gamepad2.left_bumper) {
                    rotatieGrabber.setPosition(0.35); // Setează o poziție când este apăsat left_bumper
                } else {
                    rotatieGrabber.setPosition(0.525); // Poziția implicită dacă niciun bumper nu este apăsat
                }

                // Control direct pentru gheruta cu trigger-ele
                if (gamepad2.right_trigger > 0.1) {
                    ghearaPosition = 0.45;  // Setează poziția la 0.45 când este apăsat right_trigger
                } else if (gamepad2.left_trigger > 0.1) {
                    ghearaPosition = 0.2;  // Setează poziția la 0.2 când este apăsat left_trigger
                }

                // Asigură-te că poziția este între 0.2 și 0.45
                gheruta.setPosition(ghearaPosition);

                // Control pentru extindere1 cu butoanele D-Pad
                if (gamepad2.dpad_up) {
                    extindere1.setPosition(0.52);  // Setează poziția la 0.52 când este apăsat dpad_up
                } else if (gamepad2.dpad_down) {
                    extindere1.setPosition(0.2);  // Setează poziția la 0.2 când este apăsat dpad_down
                }

                // Control pentru articulatorGrabber cu butoanele D-Pad
                if (gamepad2.dpad_right) {
                    articulatorGrabber.setPosition(0.9);  // Setează poziția la 0.9 când este apăsat dpad_right
                } else if (gamepad2.dpad_left) {
                    articulatorGrabber.setPosition(0.1);  // Setează poziția la 0.1 când este apăsat dpad_left
                }

                // Control pentru extindere2 cu butoanele A și B
                if (gamepad2.b) {
                    extindere2.setPosition(1.0);  // Setează poziția la 1 când este apăsat butonul B
                } else if (gamepad2.a) {
                    extindere2.setPosition(0.49);  // Setează poziția la 0 când este apăsat butonul A
                }
            }
        }
    });
    private final Thread PID = new Thread(new Runnable() {
        @Override
        public void run() {
            pid.enable();
            while(!stop){
                pid.setPID(pslider, islider, dslider);
                if(gamepad2.left_stick_y != 0){
                    setSetpoint = true;
                    sliderL.setPower(-gamepad2.left_stick_y);
                    sliderR.setPower(gamepad2.left_stick_y);
                }
                else if(!automatizare){
                    if(setSetpoint){
                        setSetpoint = false;
                        pid.setSetpoint(sliderL.getCurrentPosition());
                    }
                    else{
                        pidResult = pid.performPID(sliderL.getCurrentPosition());
                        sliderL.setPower(-pidResult);
                        sliderR.setPower(pidResult);
                    }
                }
                else{
                    setSetpoint = true;
                }
            }
        }
    });
    /*Aici se afla partea de program care arata cand programul se opreste, este foarte folositor pentru functionarea thread-urilor*/
    public void stop(){stop = true;}

    /*Aici se afla partea de telemetrie a robotului.
    Telemetria iti arata pe driver hub/telefon cu driver station o valoare pe care ai stabilit-o, cu un anumit text dinaintea lui*/
    @Override
    public void loop() {
        /*Exemplu de telemetrie, in care Hotel este scrisul dinainte, si trivago este valoarea, care este un string cu numele trivago :)))))*/
        telemetry.addData("motorFL:",motorFL.getCurrentPosition());
        telemetry.addData("motorBL:",motorBL.getCurrentPosition());
        telemetry.addData("motorBR:",motorBR.getCurrentPosition());
        telemetry.addData("motorFR:",motorFR.getCurrentPosition());
        telemetry.addData("sliderL:",sliderL.getPower());
        telemetry.addData("sliderL poz:",sliderL.getCurrentPosition());
        telemetry.addData("sliderR:",sliderR.getPower());
        telemetry.addData("error:",pid.getError());
        telemetry.addData("setpoint:",pid.getSetpoint());
        telemetry.addData("pos gheara:", gheruta.getPosition());
        telemetry.addData("pos articulatie gheara:", articulatorGrabber.getPosition());
        telemetry.addData("pos incheietura brat:",incheieturaBrat.getCurrentPosition());
        telemetry.addData("rotatie gheara:",rotatieGrabber.getPosition());
        telemetry.addData("extindere stanga:",extindere1.getPosition());
        telemetry.addData("extindere dreapta:",extindere2.getPosition());
        /*Aceasta functie face ca telemetria sa trimita date cat timp ruleaza programul*/
        telemetry.update();
    }
    /*Functia asta face ca toate motoarele a ruleze cu o anumita putere;
    Functiile sunt linii de cod comprimate in una singura, ceea ce este foarte fain daca vrei sa faci o secventa de linii de cod de mai multe ori. De asemenea, cand apelezi o functie, trebuie sa scrii si parametrii ei, daca exista.*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        motorFR.setPower(df1);
        motorBL.setPower(ss1);
        motorFL.setPower(sf1);
        motorBR.setPower(ds1);
    }
}
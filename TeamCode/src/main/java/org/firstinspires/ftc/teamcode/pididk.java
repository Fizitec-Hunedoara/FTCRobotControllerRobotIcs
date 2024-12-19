/* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
 */
package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.Parametri.dslider;
import static org.firstinspires.ftc.teamcode.Parametri.islider;
import static org.firstinspires.ftc.teamcode.Parametri.pslider;



import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*@TeleOp face ca programul sa apara in configuratia driver hub-ului/telefonului cu aplicatia de driver station, in partea de TeleOp. */
@TeleOp
/*Linia asta de cod incepe cu numele programului(FoundationTeleOp) si la sfarsitul liniei este tipul de program:
    OpMode = TeleOp
    LinearOpMode = Autonom
  Linia de cod va da eroare dupa ce o scrii, doar apasa pe cod, apasa pe beculetul rosu si apoi apasa pe implement methods, asta va importa functiile de init si loop.
  Functia de init se declanseaza numai o data, dar cea de loop se repeta incontinuu si este locul unde se pun functiile care misca robotul in general, sau face telemetrie in cazul asta.
 */
public class pididk extends OpMode {
    /* DcMotor este un tip de variabila cu care se declara motoarele, dar DcMotorEx este aceeasi chestie, doar cu mai multe functii*/
    public DcMotorEx motorBR,motorBL,motorFL,motorFR;
    double sm = 1;
    double y, x, rx;
    double max = 0;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;

    // public DistanceSensor distanta1, distanta2;
    boolean stop = false;
    public long cacat;
    long time_final_left = 0, time_final_right = 0, time_final = 0;

    public boolean  started_left = false, started_right = false, can_lift_intake = false;
    public double corection, error, pidResult;
    boolean encodersResetting = false;
    double pozArticulatorGrabber = 0.9;
    double ghearaPosition = 0.2;
    long lastTime,lastTimeR,lastTimeL;
    boolean extins1=false,extins2=false;

    boolean roni = false, mure = false, andrei= false, jupanu = false, maia = false;
    boolean ok, ok2;


    public double poz_artic = 0.5; //0.15

    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public FunctiiDeProgram func = new FunctiiDeProgram();


    /*Functia de init se ruleaza numai o data, se foloseste pentru initializarea motoarelor si chestii :)*/
    @Override
    public void init() {
        cacat =   System.currentTimeMillis();
        func.init(hardwareMap,telemetry,true);
        // SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/




    }
    /*Public void start se porneste o data cand se apasa pe butonul de start*/
    public void start(){
        /*Aici se pornesc thread-urile:
        Thread-urile fac parte din procesul numit multi-threading, care separa functionarea liniara a programului in mai multe bucati de program care ruleaza in acelasi timp, care se numesc thread-uri.
        De asemenea, daca folosessti aceeasi variabila in mai multe thread-uri, thread-urile se vor impiedica si se vor opri un pic.
         */
        Chassis.start();
        Systems.start();

    }
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while(!stop) {

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();

                    //Pose2d poseEstimate = drive.getPoseEstimate();
//                    telemetry.addData("x", poseEstimate.getX());
//                    telemetry.addData("y", poseEstimate.getY());
//                    telemetry.addData("heading", poseEstimate.getHeading());
//                    telemetry.update();

//                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
//                y = -gamepad1.left_stick_y;
//                x = gamepad1.left_stick_x;
//                rx = gamepad1.right_stick_x;
//
//                /* Liniile astea de cod iau niste variabile care reprezinta puterea fiecarui motor, cu ajutorul puterilor de la controller*/
//                pmotorFL = (y + x + rx);
//                pmotorBL = (y - x + rx);
//                pmotorBR = (y + x - rx);
//                pmotorFR = (y - x - rx);
//
//                /*Secventele urmatoare de cod stabilesc maximul dintre modulele puterilor motoarelor cu un anumit scop...*/
//                max = abs(pmotorFL);
//                if (abs(pmotorFR) > max) {
//                    max = abs(pmotorFR);
//                }
//                if (abs(pmotorBL) > max) {
//                    max = abs(pmotorBL);
//                }
//                if (abs(pmotorBR) > max) {
//                    max = abs(pmotorBR);
//                }
//                /*...care este punerea tuturor puterilor motoarelor sub 1, cum puterile de la motoare pot fi numai intre 1 si -1*/
//                if (max > 1) {
//                    pmotorFL /= max;
//                    pmotorFR /= max;
//                    pmotorBL /= max;
//                    pmotorBR /= max;
//                }
//                /*Aici se apeleaza functia de putere cu puterile calculate anterior ale motoarelor, si le imparte la o valoare ca robotul sa se miste cu diferite viteze.*/
//                //SLOW-MOTION
////                if (gamepad1.left_trigger>0) {
////                    sm = 1;
////                    POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
////                    //arm.setPower(poz/sm);
////                } else {
////                    //SLOWER-MOTION
////                    if (gamepad1.right_trigger>0) {
////                        sm = 2;
////                        POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
////                    } else {
////                        sm = 1;
////                        POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
////                    }
////                }
//                POWER(pmotorFR, pmotorFL, pmotorBR, pmotorBL);
            }
        }
    });

    private final Thread Systems = new Thread(new Runnable() {

        @Override
        public void run(){
            pid.enable();

            while (!stop){
                pid.setPID(pslider, islider, dslider);
                //func.sliderR.setPower(gamepad2.left_stick_y);
                //func.sliderL.setPower(gamepad2.left_stick_y);
                if (gamepad2.left_stick_y != 0.0) {
                    if (gamepad2.left_stick_y > 0.1){
                        func.sliderR.setPower(gamepad2.left_stick_y * 0.4);
                        func.sliderL.setPower(gamepad2.left_stick_y * 0.4);
                    }
                    else {
                        func.sliderR.setPower(gamepad2.left_stick_y );
                        func.sliderL.setPower(gamepad2.left_stick_y);
                    }

                    func.ceva = true;
                } else {
                    if (func.ceva) {
                        func.ceva = false;
                        pid.setSetpoint(func.sliderR.getCurrentPosition());
                    }
                    if(func.touchL.isPressed() || func.touchR.isPressed()){
                        func.sliderR.setPower(0);
                        func.sliderL.setPower(0);
                    }
                    else {
                        pidResult = pid.performPID(func.sliderR.getCurrentPosition());
                        func.sliderR.setPower(-pidResult);
                        func.sliderL.setPower(-pidResult);
                    }
                }
                func.incheieturaBrat.setPower(-gamepad2.right_stick_y * 0.6);

                if (gamepad2.right_bumper) {
                    func.rotatieGrabber.setPosition(0.7);
                }
                else if (gamepad2.left_bumper) {
                    func.rotatieGrabber.setPosition(0.35);
                }
                else {
                    func.rotatieGrabber.setPosition(0.525);
                }

                if (gamepad2.a) {
                    func.gherutaPoz = 0.45;
                }
                else if (gamepad2.b) {
                    func.gherutaPoz = 0.15;
                }
                func.gheruta.setPosition(func.gherutaPoz);
                if (gamepad2.dpad_down) {
                    func.getSpecimen();
                }
                if (gamepad2.dpad_up) {
                    func.putSpecimenOnBar();
                }
                if(gamepad2.right_trigger > 0.1 && !extins2){
                    extins2 = true;
                    func.extindere2.setPosition(1);
                    lastTime = System.currentTimeMillis();
                    while (!stop && lastTime + 200 > System.currentTimeMillis()) {
                    }
                }
                else if(gamepad2.right_trigger > 0.1 && extins2){
                    func.extindere1.setPosition(0.2);
                    extins1 = true;
                }
                if(gamepad2.left_trigger > 0.1 && extins2){
                    func.extindere2.setPosition(0.49);
                    extins2 = false;
                    lastTime = System.currentTimeMillis();
                    while (!stop && lastTime + 200 > System.currentTimeMillis()) {
                    }
                }
                else if(gamepad2.left_trigger > 0.1 && !extins2){
                    func.extindere1.setPosition(0.52);
                }
               /* if (gamepad2.a) {
                    extins2 = 0;
                }
                else if (gamepad2.b) {
                    extins2 = 1;
                }*/
                /*if (gamepad2.dpad_up) {
                    func.extindere1.setPosition(0.52);
                } else if (gamepad2.dpad_down) {
                    func.extindere1.setPosition(0.2);
                }*/

                if (gamepad2.dpad_right) {
                    func.chill();
                }
                else if (gamepad2.dpad_left) {
                    func.skibidi_dop_dop_dop();
                }
            }
        }
    });
    /*Aici se declara thread-ul cu numele systems, pentru ca contine partea de program care se ocupa de sisteme*/

    /*Aici se afla partea de program care arata cand programul se opreste, este foarte folositor pentru functionarea thread-urilor*/
    public void stop(){stop = true;}

    /*Aici se afla partea de telemetrie a robotului.
    Telemetria iti arata pe driver hub/telefon cu driver station o valoare pe care ai stabilit-o, cu un anumit text dinaintea lui*/
    @Override
    public void loop() {
        /*Exemplu de telemetrie, in care Hotel este scrisul dinainte, si trivago este valoarea, care este un string cu numele trivago :)))))*/
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//        telemetry.addData("error", pid.getError());
//        telemetry.addData("p", pid.getP());
//        telemetry.addData("d", pid.getD());
//        telemetry.addData("i", pid.getI());
        telemetry.addData("taci stanga", func.touchL.isPressed());
        telemetry.addData("taci dreapta", func.touchR.isPressed());
        telemetry.addData("poz extindere 2", func.extindere2.getPosition());
        telemetry.addData("poz extindere 1", func.extindere1.getPosition());
        telemetry.addData("extins2", extins2);
        telemetry.addData("sliderR",func.sliderR.getCurrentPosition());
//        telemetry.addData("joystick ",gamepad2.left_stick_y);
//        telemetry.addData("slider r pow ", func.sliderR.getPower());
//        telemetry.addData("slider l pow ",func.sliderL.getPower());
//        telemetry.addData("gheruta poz:",func.gheruta.getPosition());
        // telemetry.addData("distanta 1", distanta1.getDistance(DistanceUnit.MM));
        //  telemetry.addData("distanta 2", distanta2.getDistance(DistanceUnit.MM));
        telemetry.addData("brat", func.incheieturaBrat.getCurrentPosition());
        telemetry.addData("rotatie gheara:", func.rotatieGrabber.getPosition());
        telemetry.addData("articulatie gheara:", func.articulatorGrabber.getPosition());
        /*Aceasta functie face ca telemetria sa trimita date cat timp ruleaza programul*/
        telemetry.update();
    }


    /*Functia asta face ca toate motoarele a ruleze cu o anumita putere;
    Functiile sunt linii de cod comprimate in una singura, ceea ce este foarte fain daca vrei sa faci o secventa de linii de cod de mai multe ori. De asemenea, cand apelezi o functie, trebuie sa scrii si parametrii ei, daca exista.*/
    public void POWER(double df1, double sf1, double ds1, double ss1){
        func.motorFR.setPower(df1);
        func.motorBL.setPower(ss1);
        func.motorFL.setPower(sf1);
        func.motorBR.setPower(ds1);
    }
}
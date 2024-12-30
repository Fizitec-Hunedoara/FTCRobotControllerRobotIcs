/* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
 */
package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Parametri.dslider;
import static org.firstinspires.ftc.teamcode.Parametri.islider;
import static org.firstinspires.ftc.teamcode.Parametri.pslider;


import static java.lang.Math.abs;

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
    private final double slowSm = 0.4, fastSm = 1;
    private final double histInterval = 0.2;
    private boolean rtBool, ltBool, rtBoolLast = false, ltBoolLast = false;
    private final long extensorWaitTime = 200;
    private long extensorExtensionTriggerTime, extensorRetractionTriggerTime;
    private boolean extensorExtensionOverride = false, extensorRetractionOverride = false;
    private double pmotorFL, pmotorFR, pmotorBL, pmotorBR;
    double sm = 1.0;
    double y, x, rx;
    double max = 0.0;
    boolean stop = false;
    public double pidResult, pozArticulator = 0.0;
    double ghearaPosition = 0.2;
    long lastTime;
    boolean extins1 = false, extins2 = false;
    public double poz_artic = 0.5; //0.15

    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    public FunctiiDeProgram func = new FunctiiDeProgram();


    /*Functia de init se ruleaza numai o data, se foloseste pentru initializarea motoarelor si chestii :)*/
    @Override
    public void init() {
        func.init(hardwareMap, telemetry, true);

    }

    /*Public void start se porneste o data cand se apasa pe butonul de start*/
    public void start() {
        Chassis.start();
        Systems.start();

    }

    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run() {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while (!stop) {

                //thou fool
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

//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                -gamepad1.left_stick_y * sm,
//                                -gamepad1.left_stick_x * sm,
//                                -gamepad1.right_stick_x * sm
//                        )
//                );

//                drive.update();
            }
        }
    });

    private final Thread Systems = new Thread(new Runnable() {

        @Override
        public void run() {
            pid.enable();

            while (!stop) {
                pid.setPID(pslider, islider, dslider);
                if (gamepad2.left_stick_y != 0.0) {
                    if (gamepad2.left_stick_y > 0.0) {
                        func.sliderR.setPower(gamepad2.left_stick_y * 1.0);
                        func.sliderL.setPower(gamepad2.left_stick_y * 1.0);
                    }
                    else {
                        func.sliderR.setPower(gamepad2.left_stick_y);
                        func.sliderL.setPower(gamepad2.left_stick_y);
                    }
                    func.ceva = true;
                }
                else if(!func.automatizare){
                    if (func.ceva) {
                        func.ceva = false;
                        pid.setSetpoint(func.sliderR.getCurrentPosition());
                    }
                    if (func.touchL.isPressed() || func.touchR.isPressed()) {
                        func.sliderR.setPower(0);
                        func.sliderL.setPower(0);
                    }
                    else {
                        pidResult = pid.performPID(func.sliderR.getCurrentPosition());
                        func.sliderR.setPower(-pidResult);
                        func.sliderL.setPower(-pidResult);
                    }
                }
                func.incheieturaBrat.setPower(-gamepad2.right_stick_y * 2);
                func.articulatorGrabber.setPosition(func.pozArticulatorGrabber);
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
                    func.deschidere();
                }
                else if (gamepad2.b) {
                    func.inchidere();
                }
                if (gamepad2.dpad_down) {
                    func.getSpecimen();
                }
                if (gamepad2.dpad_up) {
                    func.putSpecimenOnBar();
                }
                func.gheruta.setPosition(func.gherutaPoz);

                if(gamepad2.right_trigger > (0.5 + histInterval / 2.0)) {
                    rtBool = true;
                }
                else if (gamepad2.right_trigger < (0.5 - histInterval / 2.0)){
                    rtBool = false;
                }
                if(gamepad2.left_trigger > (0.5 + histInterval / 2.0)) {
                    ltBool = true;
                }
                else if (gamepad2.left_trigger < (0.5 - histInterval / 2.0)){
                    ltBool = false;
                }

                if(rtBool != rtBoolLast){
                    if(rtBool){
                        if(func.extensorState == ExtensorState.RETRACTED){
                            func.extensorState = ExtensorState.HALF_EXTENDED;
                            extensorExtensionTriggerTime = System.currentTimeMillis();
                            extensorExtensionOverride = true;
                        }
                        else if(func.extensorState == ExtensorState.HALF_EXTENDED){
                            func.extensorState = ExtensorState.FULL_EXTENDED;
                        }
                    }
                    rtBoolLast = rtBool;
                }

                if(extensorExtensionOverride && !rtBool){
                    extensorExtensionOverride = false;
                }
                else if(extensorExtensionOverride && System.currentTimeMillis() > extensorExtensionTriggerTime + extensorWaitTime){
                    func.extensorState = ExtensorState.FULL_EXTENDED;
                }

                if(ltBool != ltBoolLast){
                    if(ltBool){
                        if(func.extensorState == ExtensorState.FULL_EXTENDED){
                            func.extensorState = ExtensorState.HALF_EXTENDED;
                            extensorRetractionTriggerTime = System.currentTimeMillis();
                            extensorRetractionOverride = true;
                        }
                        else if(func.extensorState == ExtensorState.HALF_EXTENDED){
                            func.extensorState = ExtensorState.RETRACTED;
                        }
                    }
                    ltBoolLast = ltBool;
                }

                if(extensorRetractionOverride && !ltBool){
                    extensorRetractionOverride = false;
                }
                else if(extensorRetractionOverride && System.currentTimeMillis() > extensorRetractionTriggerTime + extensorWaitTime){
                    func.extensorState = ExtensorState.RETRACTED;
                }

                func.doExtensor();

                if (gamepad2.dpad_right) {
                    func.chill();
                }
                else if (gamepad2.dpad_left) {
                    func.skibidi_dop_dop_dop();
                }
                if(gamepad2.x){
                    func.incheieturaBrat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    func.incheieturaBrat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
        }
    });
    /*Aici se declara thread-ul cu numele systems, pentru ca contine partea de program care se ocupa de sisteme*/

    /*Aici se afla partea de program care arata cand programul se opreste, este foarte folositor pentru functionarea thread-urilor*/
    public void stop() {
        stop = true;
    }

    /*Aici se afla partea de telemetrie a robotului.
    Telemetria iti arata pe driver hub/telefon cu driver station o valoare pe care ai stabilit-o, cu un anumit text dinaintea lui*/
    @Override
    public void loop() {
        /*Exemplu de telemetrie, in care Hotel este scrisul dinainte, si trivago este valoarea, care este un string cu numele trivago :)))))*/
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        telemetry.addData("taci stanga", func.touchL.isPressed());
        telemetry.addData("taci dreapta", func.touchR.isPressed());
        telemetry.addData("poz extindere 2", func.extindere2.getPosition());
        telemetry.addData("poz extindere 1", func.extindere1.getPosition());
        telemetry.addData("extins2", extins2);
        telemetry.addData("sliderR", func.sliderR.getCurrentPosition());
        telemetry.addData("brat", func.incheieturaBrat.getCurrentPosition());
        telemetry.addData("rotatie gheara:", func.rotatieGrabber.getPosition());
        telemetry.addData("articulatie gheara:", func.articulatorGrabber.getPosition());
        telemetry.addData("rtBoolean:", rtBool);
        telemetry.addData("ltBoolean:", ltBool);
        telemetry.addData("Extensor state:", func.extensorState);
        telemetry.update();
    }
}
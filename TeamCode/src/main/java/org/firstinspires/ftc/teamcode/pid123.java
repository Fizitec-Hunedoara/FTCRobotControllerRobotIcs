//package org.firstinspires.ftc.teamcode;
//
//import static java.lang.Math.abs;
//import static java.lang.Math.negateExact;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@TeleOp
//public class pid123 extends OpMode {
//    public DcMotorEx frontRight;
//    public DcMotorEx frontLeft;
//    public DcMotorEx backLeft;
//    public DcMotorEx backRight;
//    public DcMotorEx sliderL, sliderR, incheieturaBrat;
//    double sm = 1;
//    double x, y, rx;
//    double max = 0;
//    double pmotorBL;
//    double pmotorBR;
//    double pmotorFR;
//    double pmotorFL;
//    boolean stop;
//    boolean ceva =  false;
//    public double corection,error,pidResult;
//
//    double poz = 0.5;
//    double poz1 = 0.5;
//    double poz2 = 0.5;
//    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0,0,0);
//
//    //@Override
//    public void init() {
//        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
//        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");
//        incheieturaBrat = hardwareMap.get(DcMotorEx.class,"MotorMelc");
//        frontLeft = hardwareMap.get(DcMotorEx.class, "FL");
//        frontRight = hardwareMap.get(DcMotorEx.class, "FR");
//        backLeft = hardwareMap.get(DcMotorEx.class, "BL");
//        backRight = hardwareMap.get(DcMotorEx.class, "BR");
//
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
//        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        sliderR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        incheieturaBrat.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        sliderL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        incheieturaBrat.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        sliderL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        sliderR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        incheieturaBrat.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//    }
//    public void start() {
//        System.start();
//        Chassis.start();
//    }
//    public void stop(){
//        stop = true;
//    }
//    private final Thread System = new Thread(new Runnable() {
//        @Override
//        public void run() {
//            pid.enable();
//            while(!stop){
//                pid.getPID(Conf.pstatic, Conf.istatic, Conf.dstatic);
//
//
//            }
//        }
//    });
//    private final Thread Chassis = new Thread(new Runnable() {
//        @Override
//        public void run() {
//            while(!stop){
//                y = -gamepad1.left_stick_y;
//                x = gamepad1.left_stick_x;
//                rx = gamepad1.right_stick_x;
//
//                pmotorFL = (y+x+rx);
//                pmotorBL = (y-x + rx);
//                pmotorBR = (y+x-rx);
//                pmotorFR = (y-x-rx);
//
//                max = abs(pmotorFL);
//                if(abs(pmotorFR)> max){
//                    max = abs(pmotorFR);
//                }
//                if(abs(pmotorBL)> max){
//                    max = abs(pmotorBL);
//                }
//                if(abs(pmotorBR) > max){
//                    max = abs(pmotorBR);
//                }
//
//                if(max >1 ){
//                    pmotorBR /=max;
//                    pmotorBL /= max;
//                    pmotorFL /=max;
//                    pmotorFR /=max;
//                }
//                if(gamepad1.left_trigger>0){
//                    sm = 2;
//                    POWER(pmotorFR/sm,pmotorFL / sm, pmotorBR /sm , pmotorBL/ sm);
//                }
//                else{
//                    if(gamepad1.right_trigger>0){
//                        sm = 5;
//                        POWER(pmotorFR/sm,pmotorFL / sm, pmotorBR /sm , pmotorBL/ sm);
//                    }
//                    else{
//                        sm = 1;
//                        POWER(pmotorFR/sm,pmotorFL / sm, pmotorBR /sm , pmotorBL/ sm);
//                    }
//                }
//            }
//        }
//    });
//    public void POWER(double df1, double sf1, double ds1, double ss1){
//        frontRight.setPower(df1);
//        frontLeft.setPower(ss1);
//        backRight.setPower(ds1);
//        backLeft.setPower(sf1);
//    }
//    public void loop(){
//        telemetry.addData("p", pid.getP());
//        telemetry.addData("d", pid.getD());
//        telemetry.addData("i", pid.getI());
//        telemetry.addData("error", pid.getError());
//        telemetry.addData("setpoint", pid.getSetpoint());
//        telemetry.addData("position", pid.getSetpoint());
//        telemetry.update();
//    }
//}
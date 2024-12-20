package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Parametri.dslider;
import static org.firstinspires.ftc.teamcode.Parametri.islider;
import static org.firstinspires.ftc.teamcode.Parametri.pslider;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
public class AutonomRosuCos extends LinearOpMode {
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    FunctiiDeProgram func = new FunctiiDeProgram(this);
    double pidResult;
    @Override
    public void runOpMode() throws InterruptedException {
        func.init(hardwareMap,telemetry,false);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-31.1591355599, -61.0236220472, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        waitForStart();
        Systems.start();
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-53,-53,Math.toRadians(45)))
                .build();
        drive.followTrajectorySequence(ts);
        func.skibidi_dop_dop_dop_auto();
        func.kdf_auto(500);
        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-51,-51,Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(ts2);
        func.target_auto(-600,5000,func.incheieturaBrat,5000,10);
        func.initExtins = true;
        func.kdf(1000);
        func.inchidere();
    }
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run(){
            pid.enable();
            while (opModeIsActive()){
                pid.setPID(pslider, islider, dslider);
                if(func.touchL.isPressed() || func.touchR.isPressed()){
                    func.sliderR.setPower(0);
                    func.sliderL.setPower(0);
                }
                else if(!func.automatizare){
                    pid.setSetpoint(func.sliderTargetPoz);
                    pidResult = pid.performPID(func.sliderR.getCurrentPosition());
                    func.sliderR.setPower(-pidResult);
                    func.sliderL.setPower(-pidResult);
                }
                func.gheruta.setPosition(func.gherutaPoz);
                if(func.initExtins){
                    func.extindere1.setPosition(0.5);
                    func.extindere2.setPosition(0.5);
                    func.extins = true;
                    func.initExtins = false;
                }
                if(func.extins){
                    func.extindere1.setPosition(0.2);
                    func.extindere2.setPosition(0.9);
                }
            }
        }
    });
}

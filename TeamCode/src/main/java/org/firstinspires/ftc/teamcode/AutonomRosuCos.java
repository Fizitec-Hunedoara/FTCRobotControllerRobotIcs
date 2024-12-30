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
        func.extins = true;
        Systems.start();
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)))
                .build();
        drive.followTrajectorySequence(ts);
        func.pus_in_cos_auto();
        func.kdf_auto(500);
        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-50,-46,Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(ts2);
        func.extins = true;
        func.setArticulatorPoz(0.35);
        func.kdf_auto(500);
        func.target_auto(-1400,5000,func.incheieturaBrat,5000,10);
        func.kdf_auto(500);
        func.inchidere();
        func.kdf_auto(500);
        drive.followTrajectorySequence(ts);
        func.pus_in_cos_auto();
        func.kdf_auto(500);
        TrajectorySequence ts3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60,-45.5,Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(ts3);
        func.extins = true;
        func.setArticulatorPoz(0.35);
        func.kdf_auto(500);
        func.target_auto(-1400,5000,func.incheieturaBrat,5000,10);
        func.kdf_auto(500);
        func.inchidere();
        func.kdf_auto(500);
        drive.followTrajectorySequence(ts);
        func.pus_in_cos_auto();
        func.kdf_auto(500);
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
                if(func.extins){
                    func.extindere1.setPosition(0.2);
                    func.extindere2.setPosition(1);
                }
                else{
                    func.extindere1.setPosition(0.5);
                    func.extindere2.setPosition(0.5);
                }
                func.rotatieGrabber.setPosition(0.525);
            }
        }
    });
}

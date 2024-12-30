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
public class AutonomAlbastruCos extends LinearOpMode {
    public Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0, 0, 0);
    FunctiiDeProgram func = new FunctiiDeProgram(this);
    double pidResult;
    boolean extins = false;
    @Override
    public void runOpMode() throws InterruptedException {
        func.init(hardwareMap,telemetry,false);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(31.1591355599, 61.0236220472, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        waitForStart();
        Systems.start();
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(55,55,Math.toRadians(225)))
                .build();
        drive.followTrajectorySequence(ts);
        func.pus_in_cos_auto();
        func.kdf_auto(500);
        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(51,51,Math.toRadians(225)))
                .build();
        drive.followTrajectorySequence(ts2);
        func.deschidere();
        func.extins = true;
    }
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run(){
            pid.enable();
            while (opModeIsActive()) {
                pid.setPID(pslider, islider, dslider);
                if (func.touchL.isPressed() || func.touchR.isPressed()) {
                    func.sliderR.setPower(0);
                    func.sliderL.setPower(0);
                }
                else if (!func.automatizare) {
                    pid.setSetpoint(func.sliderTargetPoz);
                    pidResult = pid.performPID(func.sliderR.getCurrentPosition());
                    func.sliderR.setPower(-pidResult);
                    func.sliderL.setPower(-pidResult);

                }
                func.gheruta.setPosition(func.gherutaPoz);
                if (func.extins) {
                    func.extindere1.setPosition(0.2);
                    func.extindere2.setPosition(0.9);
                }
                else{
                    func.extindere1.setPosition(0.5);
                    func.extindere2.setPosition(0.5);
                }
            }
        }
    });
}

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Parametri.dslider;
import static org.firstinspires.ftc.teamcode.Parametri.islider;
import static org.firstinspires.ftc.teamcode.Parametri.pslider;

import android.graphics.LinearGradient;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonomAlbastruBara extends LinearOpMode {
    FunctiiDeProgram func = new FunctiiDeProgram(this);
    @Override
    public void runOpMode() throws InterruptedException {
        func.init(hardwareMap,telemetry,false);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-15.0393700787, 61.0236220472, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        waitForStart();
        Systems.start();
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0,28,Math.toRadians(90)))
                .addTemporalMarker(0.5, 0, () -> new Thread(() -> {
                    func.putSpecimenOnBar_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts);
        func.kdf_auto(200);
        func.targetSlider_auto(600,5000,5000,10);
        func.kdf_auto(200);
        func.deschidere();
        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60,60,Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(ts2);
    }
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run(){
            while (opModeIsActive()){
                func.gheruta.setPosition(func.gherutaPoz);
                func.rotatieGrabber.setPosition(0.525);
            }
        }
    });
}

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
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0,0,0);
    double pidResult = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        func.init(hardwareMap,telemetry,false);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-15.0393700787, 59.2236220472, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        waitForStart();
        Systems.start();
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(6,23,Math.toRadians(90)))
                .addTemporalMarker(0.5, 0, () -> new Thread(() -> {
                    func.putSpecimenOnBar_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts);
        func.kdf_auto(500);
        func.targetSlider_auto(0,1,5000,10);
        func.deschidere();
        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-35,28),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-47,12),Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-47,50, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(-47,18, Math.toRadians(270)))
                .splineTo(new Vector2d(-53, 18),Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-56,50, Math.toRadians(90)))
                .addDisplacementMarker(() -> new Thread(() -> {
                    func.getSpecimen_auto();
                }).start())
                .lineToLinearHeading(new Pose2d(-53, 30, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-53, 52, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(ts2);
        func.inchidere();
        func.kdf_auto(100);
        func.pozArticulatorGrabber = 1;
        TrajectorySequence ts3 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0,26,Math.toRadians(90)))
                .addTemporalMarker(0.5, 0, () -> new Thread(() -> {
                    func.putSpecimenOnBar_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts3);
        func.kdf_auto(500);
        func.getSpecimen_auto();
        TrajectorySequence ts4 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-53,30,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-53,52,Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(ts4);
        func.inchidere();
        func.kdf_auto(100);
        func.pozArticulatorGrabber = 1;
        TrajectorySequence ts5 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-4,26,Math.toRadians(90)))
                .addTemporalMarker(0.5, 0, () -> new Thread(() -> {
                    func.putSpecimenOnBar_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts5);
        func.kdf_auto(500);
        func.targetSlider_auto(0,1,5000,10);
        /*TrajectorySequence ts6 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-60,60,Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(ts6);*/
    }
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run(){
            pid.enable();
            while (opModeIsActive()){
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
                func.articulatorGrabber.setPosition(func.pozArticulatorGrabber);
                func.gheruta.setPosition(func.gherutaPoz);
                func.rotatieGrabber.setPosition(0.525);
            }
        }
    });
}

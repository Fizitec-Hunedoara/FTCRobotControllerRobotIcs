package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Parametri.dslider;
import static org.firstinspires.ftc.teamcode.Parametri.islider;
import static org.firstinspires.ftc.teamcode.Parametri.pslider;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
public class AutonomCos extends LinearOpMode {
    FunctiiDeProgram func = new FunctiiDeProgram(this);
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0,0,0);
    double pidResult = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        func.init(hardwareMap, telemetry, true);
        func.gherutaSus.setPosition(0);
        func.inchidere();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-32.0, -61.2, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        waitForStart();
        Systems.start();
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(50)))
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.targetSlider_auto(-800, 1, 5000, 10);
                    func.ghearatocos_auto();
                    func.initiala();
                    func.targetSliderJos_auto(1, 1000);
                }).start())
                .build();
        drive.followTrajectorySequence(ts);
        func.kdf_auto(500);
        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-47, -56, Math.toRadians(95)))
                .addTemporalMarker(0.8,0, () -> new Thread(() -> {
                    func.getSample();
                }).start())
                .build();
        drive.followTrajectorySequence(ts2);
        func.kdf_auto(3000);
        TrajectorySequence ts3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(40)))
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.targetSlider_auto(-800, 1, 5000, 10);
                    func.ghearatocos_auto();
                    func.initiala();
                    func.targetSliderJos_auto(1, 1000);
                }).start())
                .build();
        drive.followTrajectorySequence(ts3);
        func.kdf_auto(500);
        TrajectorySequence ts4 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-58, -57, Math.toRadians(98)))
                .addTemporalMarker(0.8,0, () -> new Thread(() -> {
                    func.getSample();
                }).start())
                .build();
        drive.followTrajectorySequence(ts4);
        func.kdf_auto(3000);
        drive.followTrajectorySequence(ts3);
        func.kdf_auto(500);
        TrajectorySequence ts5 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-58, -55.5, Math.toRadians(118)))
                .addTemporalMarker(0.8,0, () -> new Thread(() -> {
                    func.pozRotatieGhearaJos = 0.05;
                    func.getSample();
                }).start())
                .build();
        drive.followTrajectorySequence(ts5);
        func.kdf_auto(3000);
        drive.followTrajectorySequence(ts3);
        func.kdf_auto(5000);
    }


    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run(){
            pid.enable();
            while (opModeIsActive()){
                pid.setPID(pslider, islider, dslider);
                if (func.touchL.isPressed() || func.touchR.isPressed()) {
                    func.setSliderPower(0);
                }
                else if (!func.automatizare) {
                    pid.setSetpoint(func.sliderTargetPoz);
                    pidResult = pid.performPID(func.sliderR.getCurrentPosition());
                    func.setSliderPower(pidResult);
                }
                func.rotatieGherutaJos.setPosition(func.pozRotatieGhearaJos);
                func.articulatieGherutaJos.setPosition(func.pozArticulatorJos);
                func.articulatieGherutaSus.setPosition(func.pozArticulatorSus);
                func.gherutaSus.setPosition(func.pozGherutaSus);
                func.gherutaJos.setPosition(func.pozGherutaJos);
                func.rotatiefata.setPosition(func.pozRotatie);
                func.setArmPosition(func.pozArm);
                func.setExtinderePoz();
            }
        }
    });
}


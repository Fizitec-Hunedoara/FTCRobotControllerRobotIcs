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
public class AutonomAlbastruBara extends LinearOpMode {
    FunctiiDeProgram func = new FunctiiDeProgram(this);
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0,0,0);
    double pidResult = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        func.init(hardwareMap,telemetry,true);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15.1, -61.2, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        waitForStart();
        Systems.start();
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-6,-24,Math.toRadians(270)))
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.ghearapozbara_auto(false);
                    func.kdf_auto(700);
                    func.puspebara_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts);
        func.kdf_auto(500);
        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(35,-28),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(47,-12),Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(47, -50,Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                .lineToLinearHeading(new Pose2d(47, -12, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                .addDisplacementMarker(() -> new Thread(() -> {
                    func.gardtogheara_auto();
                }).start())
                .lineToLinearHeading(new Pose2d(57, -12, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                .lineToLinearHeading(new Pose2d(57, -50, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                /*.lineToLinearHeading(new Pose2d(47,-18, Math.toRadians(90)))
                .splineTo(new Vector2d(53, -18),Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(56,-50, Math.toRadians(270)))*/
                .lineToLinearHeading(new Pose2d(38, -50, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                .lineToLinearHeading(new Pose2d(38, -62, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                .build();
        drive.followTrajectorySequence(ts2);
        func.inchidere();
        func.kdf_auto(100);
        TrajectorySequence ts3 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-3,-24,Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.ghearapozbara_auto(true);
                    func.kdf_auto(700);
                    func.puspebara_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts3);
        func.kdf_auto(500);
        func.gardtogheara_auto();
        TrajectorySequence ts4 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(43,-64,Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                .build();
        drive.followTrajectorySequence(ts4);
        func.inchidere();
        func.kdf_auto(100);
        TrajectorySequence ts5 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-7,-24,Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.ghearapozbara_auto(true);
                    func.kdf_auto(700);
                    func.puspebara_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts5);
        func.kdf_auto(500);
        func.gardtogheara_auto();
        drive.followTrajectorySequence(ts4);
        func.kdf_auto(200);
        func.inchidere();
        TrajectorySequence ts7 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-1,-24,Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.ghearapozbara_auto(true);
                    func.kdf_auto(700);
                    func.puspebara_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts7);
        func.kdf_auto(500);
        TrajectorySequence ts6 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(60,-60,Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(80.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80.0)
                )
                .build();
        drive.followTrajectorySequence(ts6);
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
                func.articulatieGheruta.setPosition(func.pozArticulator);
                func.gheruta.setPosition(func.pozGheruta);
                func.armL.setPosition(func.pozArm);
                func.armR.setPosition(func.pozArm);
                func.extindereL.setPosition(0.01);
                func.extindereR.setPosition(0.99);
            }
        }
    });
}

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
public class AutonomBara extends LinearOpMode {
    FunctiiDeProgram func = new FunctiiDeProgram(this);
    Pid_Controller_Adevarat pid = new Pid_Controller_Adevarat(0,0,0);
    double pidResult = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {
        func.init(hardwareMap, telemetry, true);
        func.gherutaSus.setPosition(0);
        func.inchidere();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(15.1, -61.2, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence ts3 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-2, -28, Math.toRadians(270)))
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.ghearapozbara_auto(false);
                    func.kdf_auto(450);
                    func.puspebara_auto();
                }).start())
                .build();
        TrajectorySequence ts7 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(35,-28),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(41, -9), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(46,-12),Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(46, -51,Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(85.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(85.0)
                )
                .lineToLinearHeading(new Pose2d(45, -12, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(90.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(90.0)
                )
                .splineToConstantHeading(new Vector2d(59, -22), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(56, -51, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(85.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(85.0)
                )
                .lineToLinearHeading(new Pose2d(56, -9, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(90.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(90.0)
                )
                .splineToConstantHeading(new Vector2d(60, -9), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(65, -12),Math.toRadians(270))
                .addDisplacementMarker(() -> new Thread(() -> {
                    func.gardtogheara_auto();
                }).start())
                .lineToLinearHeading(new Pose2d(65, -41, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(85.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(85.0)
                )
                .splineToConstantHeading(new Vector2d(40, -60),Math.toRadians(270))
                .build();
        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(new Vector2d(35,-28),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(41, -11),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47,-14),Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(47, -51,Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(85.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(85.0)
                )
                .lineToLinearHeading(new Pose2d(47, -12, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(90.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(90.0)
                )
                .splineToConstantHeading(new Vector2d(52, -9),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(57, -12), Math.toRadians(270))
                .lineToLinearHeading(new Pose2d(57, -51, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(85.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(85.0)
                )
                .lineToLinearHeading(new Pose2d(57, -9, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(90.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(90.0)
                )
                .splineToConstantHeading(new Vector2d(61, -9), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(65, -12),Math.toRadians(270))
                .addDisplacementMarker(() -> new Thread(() -> {
                    func.gardtogheara_auto();
                }).start())
                .lineToLinearHeading(new Pose2d(65, -41, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(85.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(85.0)
                )
                .splineToConstantHeading(new Vector2d(40, -58),Math.toRadians(270))
                .build();
        waitForStart();
        Systems.start();
        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-8, -28, Math.toRadians(270)))
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.kdf_auto(350);
                    func.ghearapozbara_auto(false);
                    func.kdf_auto(500);
                    func.puspebara_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts);
        if(func.getBatteryVoltage() > 13.5) {
            drive.followTrajectorySequence(ts2);
        }
        else{
            drive.followTrajectorySequence(ts7);
        }
        func.inchidere();
        drive.followTrajectorySequence(ts3);
        TrajectorySequence ts4 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(42, -59, Math.toRadians(270)))
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.gardtogheara_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts4);
        func.inchidere();
        TrajectorySequence ts5 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(6, -28, Math.toRadians(270)))
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.ghearapozbara_auto(false);
                    func.kdf_auto(450);
                    func.puspebara_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts5);
        drive.followTrajectorySequence(ts4);
        TrajectorySequence ts6 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(3, -28, Math.toRadians(270)))
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.ghearapozbara_auto(false);
                    func.kdf_auto(450);
                    func.puspebara_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts6);
        drive.followTrajectorySequence(ts4);
        TrajectorySequence ts8 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(8, -28, Math.toRadians(270)))
                .addTemporalMarker(0, 0, () -> new Thread(() -> {
                    func.ghearapozbara_auto(false);
                    func.kdf_auto(450);
                    func.puspebara_auto();
                }).start())
                .build();
        drive.followTrajectorySequence(ts8);
        drive.followTrajectorySequence(ts4);
    }
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run(){
            //pid.enable();
            while (opModeIsActive()){
                /*pid.setPID(pslider, islider, dslider);
                if (func.touchL.isPressed() || func.touchR.isPressed()) {
                    func.sliderR.setPower(0);
                    func.sliderL.setPower(0);
                }
                else if (!func.automatizare) {
                    pid.setSetpoint(func.sliderTargetPoz);
                    pidResult = pid.performPID(func.sliderR.getCurrentPosition());
                    func.sliderR.setPower(-pidResult);
                    func.sliderL.setPower(-pidResult);
                }*/
                func.articulatieGherutaSus.setPosition(func.pozArticulatorSus);
                func.gherutaSus.setPosition(func.pozGherutaSus);
                func.gherutaJos.setPosition(func.pozGherutaJos);
                func.armL.setPosition(func.pozArm);
                func.armR.setPosition(func.pozArm);
                func.extindereL.setPosition(0.05);
                func.extindereR.setPosition(0.975);
            }
        }
    });
}

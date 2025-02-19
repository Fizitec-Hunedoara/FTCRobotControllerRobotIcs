package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.11382049845574, 50.11382049845574, Math.toRadians(308.5209343088889), Math.toRadians(346.43243243243245), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-32, -61.2, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(50)))
                        .splineToLinearHeading(new Pose2d(-50, -57,Math.toRadians(90)), Math.toRadians(90))
                        .lineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(50)))
                        .lineToLinearHeading(new Pose2d(-58, -57, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-56, -54, Math.toRadians(50)))
                        .lineToLinearHeading(new Pose2d(-58, -57, Math.toRadians(110)))
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
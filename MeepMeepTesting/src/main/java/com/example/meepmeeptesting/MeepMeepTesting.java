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
                .setConstraints(50.11382049845574, 50.11382049845574, Math.toRadians(319.8315), Math.toRadians(346.43243243243245), 11.47)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(15.1, -61.2, 270))
                        .lineToLinearHeading(new Pose2d(-6,-23,Math.toRadians(270)))
                        .splineToConstantHeading(new Vector2d(49,-37),Math.toRadians(90))

                        /*.splineToConstantHeading(new Vector2d(47,-12),Math.toRadians(270))
                        .lineToLinearHeading(new Pose2d(47, -50,Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(47, -12, Math.toRadians(270)))
                        .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(270))
                        .lineToLinearHeading(new Pose2d(60, -50, Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(36, -50, Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(36, -62, Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(-3,-23,Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(36,-62,Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(3,-23,Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(36,-62,Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(-6,-23,Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(60,-60,Math.toRadians(270)))*/
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
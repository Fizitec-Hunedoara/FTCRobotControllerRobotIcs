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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .lineToLinearHeading(new Pose2d(-8, -24, Math.toRadians(270)))
                        .splineToConstantHeading(new Vector2d(35,-28),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(47,-12),Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(47,-51),Math.toRadians(270))
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(47,-12),Math.toRadians(270))
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(57,-12),Math.toRadians(270))
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(57,-51),Math.toRadians(270))
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(57,-12),Math.toRadians(270))
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(61,-12),Math.toRadians(270))
                        .waitSeconds(0.5)
                        .splineToConstantHeading(new Vector2d(61,-51),Math.toRadians(270))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
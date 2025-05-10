package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//pos depot rosu: 61.0236220472,15.0393700787
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(53.17330064499293, 53.17330064499293, Math.toRadians(314.3483), Math.toRadians(322.7499076212471), 11.03)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-31.1591355599, -61.0236220472, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-56,-56,Math.toRadians(45)))
                        .lineToLinearHeading(new Pose2d(-50,-46,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-60,-46,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-60,-46,Math.toRadians(110)))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
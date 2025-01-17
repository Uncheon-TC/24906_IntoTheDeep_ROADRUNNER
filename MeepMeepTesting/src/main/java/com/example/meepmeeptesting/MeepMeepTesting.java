package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 11.35872513)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-20, 61, Math.PI / 2))
                        .setTangent(3 * Math.PI / 2)


                        .splineToLinearHeading(new Pose2d(-12, 35, Math.PI / 2), 3 * Math.PI / 2) //1st chamber
                        .waitSeconds(0.5)
                        .splineToSplineHeading(new Pose2d(-35.5, 36, 3 * Math.PI / 2), 3 * Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(-36, 8, 3 * Math.PI / 2), 3 * Math.PI / 2)
                        .splineToLinearHeading(new Pose2d(-46, 11, 3 * Math.PI / 2), Math.PI / 2)
                        .lineToY(50)
                        .setTangent(3 * Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-46, 11), 3 * Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-54, 16), Math.PI / 2)
                        .lineToY(50)
                        .setTangent(3 * Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-54, 11), 3 * Math.PI / 2)
                        .splineToConstantHeading(new Vector2d(-60, 16), Math.PI / 2)
                        .lineToY(40)
                        .splineToSplineHeading(new Pose2d(-47, 60, 3 * Math.PI / 2), Math.PI / 2) //clip pickup
                        .waitSeconds(0.5)
                        .setTangent(3 * Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(-8, 35, Math.PI / 2), 3 * Math.PI / 2) //2nd chamber
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(-47, 60, 3 * Math.PI / 2), Math.PI / 2)
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(-4, 35, Math.PI / 2), 3 * Math.PI / 2) //3rd chamber
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(-47, 60, 3 * Math.PI / 2), Math.PI / 2)
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(0, 35, Math.PI / 2), 3 * Math.PI / 2) //4th chamber
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
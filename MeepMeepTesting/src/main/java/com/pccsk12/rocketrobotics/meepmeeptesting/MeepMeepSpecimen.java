package com.pccsk12.rocketrobotics.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepSpecimen {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 80, Math.toRadians(180), Math.toRadians(180), 15.5)
            .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8.75, -63, Math.toRadians(90)))
            .waitSeconds(0.6)
            .strafeTo(new Vector2d(8.75, -38))
            .waitSeconds(1) // clip
            .strafeTo(new Vector2d(8.75, -48))
            .strafeToLinearHeading(new Vector2d(34, -48), Math.toRadians(-89.99))
            .strafeTo(new Vector2d(36, -48))
            .strafeTo(new Vector2d(45, -12))
            .endTrajectory() // intake
            .strafeTo(new Vector2d(45, -57 + 3))
            .waitSeconds(0.5)
            .strafeTo(new Vector2d(45, -57))
            .waitSeconds(0.7)
            .endTrajectory() // stop intake
            .strafeTo(new Vector2d(48, -40))
            .setTangent(Math.toRadians(180)).splineToSplineHeading(new Pose2d(4.4, -38, Math.toRadians(90)), Math.toRadians(90))
            .waitSeconds(1) // clip
            .strafeTo(new Vector2d(4.4, -48))
            .setTangent(0) .splineToSplineHeading(new Pose2d(45, -57 + 3, Math.toRadians(-89.99)), Math.toRadians(-89.99))
            .waitSeconds(0.5)
            .strafeTo(new Vector2d(45, -57))
            .waitSeconds(0.7)
            .endTrajectory() // stop intake
            .setTangent(Math.toRadians(180)).splineToSplineHeading(new Pose2d(0.4, -38, Math.toRadians(90)), Math.toRadians(90))
            .waitSeconds(1) // clip
            .strafeTo(new Vector2d(0.4, -48))
            .endTrajectory()
            .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
//            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}
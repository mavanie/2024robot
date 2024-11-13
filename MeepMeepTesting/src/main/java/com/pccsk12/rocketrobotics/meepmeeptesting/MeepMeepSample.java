package com.pccsk12.rocketrobotics.meepmeeptesting;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepSample {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 80, Math.toRadians(180), Math.toRadians(180), 15.5)
            .build();

        VelConstraint slow = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(7),
            new AngularVelConstraint(1.8)
        ));

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35.25, -63.75, Math.toRadians(90)))
            .strafeToSplineHeading(new Vector2d(-49, -53), Math.toRadians(-135), slow)
            .waitSeconds(2) // drop
            .strafeToSplineHeading(new Vector2d(-46, -46), Math.toRadians(91), slow)
            .waitSeconds(4) // pick up
            .lineToY(-46 + 4, slow)
            .waitSeconds(2) // retract slides
            .strafeToSplineHeading(new Vector2d(-49, -53), Math.toRadians(-135), slow)
            .waitSeconds(2) // drop
            .strafeToSplineHeading(new Vector2d(-56, -46), Math.toRadians(92), slow)
            .waitSeconds(4) // pick up
            .lineToY(-46 + 4, slow)
            .waitSeconds(2) // retract slides
            .strafeToSplineHeading(new Vector2d(-49, -53), Math.toRadians(-135), slow)
            .waitSeconds(2) // drop
            .endTrajectory()
            .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
//            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}
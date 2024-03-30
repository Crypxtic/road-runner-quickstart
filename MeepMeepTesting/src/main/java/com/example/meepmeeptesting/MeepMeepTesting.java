package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                .strafeToSplineHeading(new Vector2d(12, -35.65), Math.toRadians(90)) //use old
                .strafeToSplineHeading(new Vector2d(47.24, -35.47), Math.toRadians(180.00))
                .endTrajectory()//use old
                .strafeToSplineHeading(new Vector2d(46.24, -37.47), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(15, -58), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-40, -58), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58, -40), Math.toRadians(90))
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(-57, -43), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-40, -58), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(15, -58))
                .splineToConstantHeading(new Vector2d(47.24, -35.47), Math.toRadians(80))
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(46.24, -37.47), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(15, -58), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-40, -58), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58, -40), Math.toRadians(90))
                .endTrajectory()
                .strafeToSplineHeading(new Vector2d(-57, -43), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-40, -58), Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(15, -58))
                .splineToConstantHeading(new Vector2d(47.24, -35.47), Math.toRadians(80))
                .endTrajectory()
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
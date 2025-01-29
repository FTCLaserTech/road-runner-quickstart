package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));
        Pose2d toSubmursible = new Pose2d(-8,29,Math.toRadians(270));
        Pose2d backUpFromSubmursible = new Pose2d(18,26,Math.toRadians(270));
        //Pose2d backUpFromSubmursible2 = new Pose2d(18,29,Math.toRadians(270));
        //Pose2d lineUpForSweep = new Pose2d(30,50,Math.toRadians(270));
        Pose2d forward1 = new Pose2d(18,38,Math.toRadians(270));
        Pose2d slideOver1 = new Pose2d(31,44,Math.toRadians(270));
        Pose2d sweep = new Pose2d(31,15,Math.toRadians(270));
        Pose2d backTo1 = new Pose2d(31,38,Math.toRadians(270));
        Pose2d slideOver2 = new Pose2d(43,40,Math.toRadians(270));
        Pose2d sweep2 = new Pose2d(42,14.5,Math.toRadians(270));
        Pose2d backTo2 = new Pose2d(40,38,Math.toRadians(270));
        Pose2d slideOver3 = new Pose2d(55,42,Math.toRadians(270));
        Pose2d sweep3 = new Pose2d(54,12,Math.toRadians(270));
        Pose2d sweep3a = new Pose2d(54,15,Math.toRadians(270));
        Pose2d lineUpForWallSlide = new Pose2d(35,-5,Math.toRadians(180));
        Pose2d wallSlide = new Pose2d(51,-3,Math.toRadians(180));
        Pose2d lineUpForWallSlide2 = new Pose2d(-30,-9,Math.toRadians(180));
        Pose2d wallSlide2 = new Pose2d(-1,-9,Math.toRadians(180));
        //Pose2d lineUpForWallSlide3 = new Pose2d(-20,-9,Math.toRadians(180));
        //Pose2d wallSlide3 = new Pose2d(2,-9,Math.toRadians(180));
        //Pose2d lineUpForWallSlide4 = new Pose2d(-20,-9,Math.toRadians(180));
        //Pose2d wallSlide4 = new Pose2d(2,-9,Math.toRadians(180));
        Pose2d toSubmursible2 = new Pose2d(-65,34,Math.toRadians(265));
        Pose2d toSubmursible3 = new Pose2d(-65,34,Math.toRadians(265));
        Pose2d toSubmursible4 = new Pose2d(-65,34,Math.toRadians(265));
        Pose2d toSubmursible5 = new Pose2d(-65,34,Math.toRadians(265));
        Pose2d park = new Pose2d(-18,-2,Math.toRadians(265));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-8, 29, Math.toRadians(270)))

                        .splineToLinearHeading(backUpFromSubmursible, Math.toRadians(90))
                        //.splineToLinearHeading(backUpFromSubmursible2, Math.toRadians(90), new TranslationalVelConstraint(55.0))
                        .splineToLinearHeading(forward1, Math.toRadians(90))
                        .splineToLinearHeading(slideOver1, Math.toRadians(180))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
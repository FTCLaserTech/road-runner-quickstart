package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Config
@Autonomous(group = "a")

public class DumpandPark extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0, Math.toRadians(initialRotation));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        drive.PARAMS.headingGain = 7.0;

        extras.initArm();

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
        drive.odo.update();
        telemetry.addData("ODO heading", drive.odo.getHeading());
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {
            safeWaitSeconds(0.010);
        }

        // put the preloaded sample in the high basket
        VelConstraint baseVelConstraint=new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0),
                new AngularVelConstraint(Math.toRadians(100))));




        Pose2d nearPole = new Pose2d(-15,4, Math.toRadians(270));
        Pose2d toPole = new Pose2d(-20.5,6, Math.toRadians(405));
        Pose2d avoidSamples = new Pose2d(10,23, Math.toRadians(450));
        Pose2d lineUpForPark = new Pose2d(10,45, Math.toRadians(360));
        Pose2d park = new Pose2d(20,45, Math.toRadians(360));






        Action DriveToBasketPole = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(nearPole.position, nearPole.heading, baseVelConstraint)
                .strafeToLinearHeading(toPole.position, toPole.heading, baseVelConstraint, new ProfileAccelConstraint(-40,40))
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToBasketPole,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new SleepAction(1.6),
                        new InstantAction(() -> extras.elevatorHighBasket()),
                        new SleepAction(1.8),
                        new InstantAction(() -> extras.sampleDump()),
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.samplePickup()),
                        new SleepAction(0.5)
                )
        ));

        Action driveToBasketPole = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(avoidSamples.position, avoidSamples.heading, new TranslationalVelConstraint(10.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                driveToBasketPole,
                new SequentialAction(
                        new SleepAction(0.5),
                        new InstantAction(() -> extras.elevatorDown())
                        )
        ));

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(lineUpForPark.position, lineUpForPark.heading, new TranslationalVelConstraint(10.0))
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(park.position, park.heading, new TranslationalVelConstraint(10.0))
                        .build());





        extras.saveAutoStartRotation(drive.odo.getHeading() + initialRotation - PI/2);
    }

    public void safeWaitSeconds(double time)
    {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time)
        {
            ;
        }
    }
}
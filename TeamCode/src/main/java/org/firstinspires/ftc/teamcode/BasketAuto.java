package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.CompositeVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.reflect.Array;
import java.util.Arrays;

@Config
@Autonomous(group = "a")

public class BasketAuto extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 90;
        Pose2d initPose = new Pose2d(0,0, Math.toRadians(initialRotation));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

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
                new TranslationalVelConstraint(15.0),
                new AngularVelConstraint(Math.toRadians(45))));
        Pose2d nearPole = new Pose2d(-15,4, Math.toRadians(90));
        Pose2d toPole = new Pose2d(-19,6, Math.toRadians(45));
        Action DriveToBasketPole = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(nearPole.position, nearPole.heading, baseVelConstraint)
                //.turnTo(45)
                .strafeToLinearHeading(toPole.position, toPole.heading, baseVelConstraint, new ProfileAccelConstraint(-40,40))
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToBasketPole,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new InstantAction(() -> extras.elevatorHighBasket()),
                        new SleepAction(3),
                        new InstantAction(() -> extras.sampleDump()),
                        new SleepAction(1),
                        new InstantAction(() -> extras.samplePickup()),
                        new SleepAction(2)
                )
        ));

        telemetry.addLine("Before 1 updatePoseEstimate");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
        drive.odo.update();
        telemetry.addData("ODO heading", drive.odo.getHeading());
        telemetry.update();

        safeWaitSeconds(2);
        drive.updatePoseEstimate();

        telemetry.addLine("After 1 updatePoseEstimate");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
        drive.odo.update();
        telemetry.addData("ODO heading", drive.odo.getHeading());
        telemetry.update();

        safeWaitSeconds(2);


        Pose2d toFirstSample = new Pose2d(19,14, Math.toRadians(45));
        Action DriveToFirstSample = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toFirstSample.position, toFirstSample.heading, new TranslationalVelConstraint(15.0))
                .turnTo(90)
                //.strafeToLinearHeading(toPole.position, toPole.heading, new TranslationalVelConstraint(15.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToFirstSample,
                new SequentialAction(
                        new SleepAction(1),
                        new InstantAction(() -> extras.elevatorDown()),
                        new SleepAction(3),
                        new InstantAction(() -> extras.armExtend()),
                        new SleepAction(2.0)
                )

        ));
        telemetry.addLine("Before 2 updatePoseEstimate");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
        drive.odo.update();
        telemetry.addData("ODO heading", drive.odo.getHeading());
        telemetry.update();

        safeWaitSeconds(1);
        drive.updatePoseEstimate();

        telemetry.addLine("After 2 updatePoseEstimate");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
        drive.odo.update();
        telemetry.addData("ODO heading", drive.odo.getHeading());
        telemetry.update();
/*
        Pose2d firstsample = new Pose2d(15,18, Math.toRadians(90));
        Action FirstSampleAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(firstsample.position, firstsample.heading, new TranslationalVelConstraint(15.0))
                .turnTo(90)
                //.strafeToLinearHeading(toPole.position, toPole.heading, new TranslationalVelConstraint(15.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                FirstSampleAction,
                new SequentialAction(
                        new InstantAction(() -> extras.intakeIn()),
                        new SleepAction(3),
                        new InstantAction(() -> extras.intakeOff()),
                        new SleepAction(2.0)
                )

        ));

 */

        safeWaitSeconds(5);

        //pick up the 1st sample on the field and put it in the high basket

        //pick up the 2nd sample on the field and put it in the high basket

        //pick up the 3rd sample on the field and put it in the high basket

        //Park

        // Save the ending location
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


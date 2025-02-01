package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@Autonomous(group = "a")
@Disabled

public class Sweep3PinPoint extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));
        Pose2d toSubmursible = new Pose2d(-12,30,Math.toRadians(270));
        Pose2d backUpFromSubmursible = new Pose2d(15,23,Math.toRadians(400));
        Pose2d slideOver2 = new Pose2d(20,23,Math.toRadians(400));
        Pose2d slideOver3 = new Pose2d(25,23,Math.toRadians(400));

        Pose2d lineUpForWallSlide = new Pose2d(10,-3,Math.toRadians(180));
        Pose2d wallSlide = new Pose2d(51,-3,Math.toRadians(180));
        Pose2d lineUpForWallSlide2 = new Pose2d(-20,-7,Math.toRadians(180));
        Pose2d wallSlide2 = new Pose2d(2,-5,Math.toRadians(180));
        Pose2d lineUpForWallSlide3 = new Pose2d(-20,-7,Math.toRadians(180));
        Pose2d wallSlide3 = new Pose2d(2,-7,Math.toRadians(180));
        Pose2d toSubmursible2 = new Pose2d(-65,35,Math.toRadians(265));
        Pose2d toSubmursible3 = new Pose2d(-68,35,Math.toRadians(265));
        Pose2d toSubmursible4 = new Pose2d(-71,35,Math.toRadians(265));
        Pose2d park = new Pose2d(-18,-2,Math.toRadians(265));


        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);
        drive.PARAMS.headingGain = 5;

        extras.initArm();
        extras.tailUp();

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {
            safeWaitSeconds(0.01);
        }

        // hang pre-loaded sample in the high chamber
        Action DriveToNearSubmursibleAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toSubmursible.position, toSubmursible.heading, new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-50,50))
                // move to submersible
                //.strafeToLinearHeading(new Vector2d(-15,29), Rotation2d.exp(270), new TranslationalVelConstraint(15.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToNearSubmursibleAction,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new InstantAction(() -> extras.elevatorHighChamber()),
                        new SleepAction(1.4),
                        new InstantAction(() -> extras.elevatorDown())
                )
        ));

        safeWaitSeconds(0.15);

        Action LineUpForSweepAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(backUpFromSubmursible.position, backUpFromSubmursible.heading, new TranslationalVelConstraint(20.0), new ProfileAccelConstraint(-50,50))
                // move to submersible
                //.strafeToLinearHeading(new Vector2d(-15,29), Rotation2d.exp(270), new TranslationalVelConstraint(15.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                LineUpForSweepAction,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armExtend())
                )
        ));


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(320)
                        .strafeToLinearHeading(slideOver2.position, slideOver2.heading, new TranslationalVelConstraint(50.0))
                        .turnTo(320)
                        .strafeToLinearHeading(slideOver3.position, slideOver3.heading, new TranslationalVelConstraint(50.0))
                        .turnTo(300)
                        .build());


        /*
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(lineUpForWallSlide.position, lineUpForWallSlide.heading, new TranslationalVelConstraint(50.0))
                        .build());

        safeWaitSeconds(0.4);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(wallSlide.position, wallSlide.heading, new TranslationalVelConstraint(50.0))
                        .build());

        drive.odo.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES, 180-initialRotation));
        drive.pose = new Pose2d(0,0,Math.toRadians(180));

        Action DriveToNearSubmursibleAction2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toSubmursible2.position, toSubmursible2.heading, new TranslationalVelConstraint(50.0))
                // move to submersible
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToNearSubmursibleAction2,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new InstantAction(() -> extras.elevatorHighChamber()),
                        new SleepAction(2.1),
                        new InstantAction(() -> extras.elevatorDown()),
                        new SleepAction(0.15)
                )
        ));


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(lineUpForWallSlide2.position, lineUpForWallSlide2.heading, new TranslationalVelConstraint(50.0))
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(wallSlide2.position, wallSlide2.heading, new TranslationalVelConstraint(50.0))
                        .build());

        drive.odo.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES, 180-initialRotation));
        drive.pose = new Pose2d(0,0,Math.toRadians(180));

        Action DriveToNearSubmursibleAction3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toSubmursible3.position, toSubmursible3.heading, new TranslationalVelConstraint(50.0))
                // move to submersible
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToNearSubmursibleAction3,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new InstantAction(() -> extras.elevatorHighChamber()),
                        new SleepAction(2.1),
                        new InstantAction(() -> extras.elevatorDown()),
                        new SleepAction(0.15)
                )
        ));

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(lineUpForWallSlide3.position, lineUpForWallSlide3.heading, new TranslationalVelConstraint(50.0))
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(wallSlide3.position, wallSlide3.heading, new TranslationalVelConstraint(50.0))
                        .build());

        drive.odo.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES, 180-initialRotation));
        drive.pose = new Pose2d(0,0,Math.toRadians(180));

        Action DriveToNearSubmursibleAction4 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toSubmursible4.position, toSubmursible4.heading, new TranslationalVelConstraint(50.0))
                // move to submersible
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToNearSubmursibleAction4,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new InstantAction(() -> extras.elevatorHighChamber()),
                        new SleepAction(2.1),
                        new InstantAction(() -> extras.elevatorDown()),
                        new SleepAction(0.15)
                )
        ));

        Action parkAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(park.position, park.heading, new TranslationalVelConstraint(50.0))
                // move to submersible
                .build();

        Actions.runBlocking(new ParallelAction(
                parkAction,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armRetract())
                )
        ));
    */


        /*
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(320)
                        .build());
        */
        safeWaitSeconds(20);

        // Save the ending location
        extras.saveAutoStartRotation(drive.odo.getHeading()+ initialRotation - PI/2);
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


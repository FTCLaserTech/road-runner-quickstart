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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@Autonomous(group = "a")


public class SpecimensAuto5 extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));
        Pose2d toSubmursible = new Pose2d(-13,30,Math.toRadians(270));
        Pose2d backUpFromSubmursible = new Pose2d(20,26,Math.toRadians(270));
        Pose2d forward1 = new Pose2d(20,38,Math.toRadians(270));
        Pose2d slideOver1 = new Pose2d(34,50,Math.toRadians(270));
        Pose2d sweep = new Pose2d(33,18,Math.toRadians(270));
        Pose2d backTo1 = new Pose2d(32,45,Math.toRadians(270));
        Pose2d slideOver2 = new Pose2d(50,45,Math.toRadians(270));
        Pose2d sweep2 = new Pose2d(51,16,Math.toRadians(270));
        Pose2d backTo2 = new Pose2d(45,48,Math.toRadians(270));
        Pose2d slideOver3 = new Pose2d(60,48,Math.toRadians(270));
        Pose2d sweep3 = new Pose2d(70,10,Math.toRadians(270));
        Pose2d sweep3a = new Pose2d(65,14,Math.toRadians(270));
        Pose2d lineUpForWallSlide = new Pose2d(55,-15,Math.toRadians(180));
        Pose2d wallSlide = new Pose2d(70,-15,Math.toRadians(180));
        Pose2d lineUpForWallSlide2 = new Pose2d(-20,-7,Math.toRadians(180));
        Pose2d wallSlide2 = new Pose2d(0,-5,Math.toRadians(180));
        Pose2d toSubmursible2 = new Pose2d(-60,37,Math.toRadians(265));
        Pose2d toSubmursible3 = new Pose2d(-60,37,Math.toRadians(265));
        Pose2d toSubmursible4 = new Pose2d(-60,37,Math.toRadians(265));
        Pose2d toSubmursible5 = new Pose2d(-60,37,Math.toRadians(265));
        Pose2d park = new Pose2d(-18,-2,Math.toRadians(265));

        Pose2D ppPos;

        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        extras.initArm();
        extras.tailUp();
        extras.armRetract();

        while (!gamepad2.dpad_left)
        {
            sleep(100);
            telemetry.addLine("Align robot to wall");
            telemetry.addLine("GP2 DPad Left to exit");
            telemetry.update();
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        ppPos = drive.odo.getPosition();
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

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(backUpFromSubmursible, Math.toRadians(90), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        //.splineToLinearHeading(backUpFromSubmursible2, Math.toRadians(90), new TranslationalVelConstraint(55.0))
                        .splineToSplineHeading(forward1, Math.toRadians(90), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        .splineToLinearHeading(slideOver1, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        .splineToLinearHeading(sweep, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        .splineToLinearHeading(backTo1, Math.toRadians(0), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        .splineToLinearHeading(slideOver2, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        .splineToLinearHeading(sweep2, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        .splineToLinearHeading(backTo2, Math.toRadians(0), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        .splineToLinearHeading(slideOver3, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        .splineToLinearHeading(sweep3, Math.toRadians(0), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(sweep3a, Math.toRadians(180), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70,70))
                        .build());



        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(lineUpForWallSlide, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                        .strafeToLinearHeading(wallSlide.position, wallSlide.heading, new TranslationalVelConstraint(55.0))
                        .build());

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
                        .splineToLinearHeading(lineUpForWallSlide2, Math.toRadians(0), new TranslationalVelConstraint(50.0))
                        .splineToLinearHeading(wallSlide2, Math.toRadians(0), new TranslationalVelConstraint(50.0))
                        .build());


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
                        .splineToLinearHeading(lineUpForWallSlide2, Math.toRadians(0), new TranslationalVelConstraint(50.0))
                        .splineToLinearHeading(wallSlide2, Math.toRadians(0), new TranslationalVelConstraint(50.0))
                        .build());

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


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(lineUpForWallSlide2, Math.toRadians(0), new TranslationalVelConstraint(50.0))
                        .splineToLinearHeading(wallSlide2, Math.toRadians(0), new TranslationalVelConstraint(50.0))
                        .build());

        drive.pose = new Pose2d(0,0,Math.toRadians(180));

        Action DriveToNearSubmursibleAction5 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toSubmursible5.position, toSubmursible5.heading, new TranslationalVelConstraint(50.0))
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



        /*
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turnTo(320)
                        .build());
        */
        safeWaitSeconds(5);


        // Hang #2

        // Hang #3

        // Hang #4

        // Hang #5

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


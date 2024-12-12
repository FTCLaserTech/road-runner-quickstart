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

public class BasketRevAuto extends LinearOpMode
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
                        new SleepAction(1),
                        new InstantAction(() -> extras.samplePickup()),
                        new SleepAction(0.5)
                )
        ));

        safeWaitSeconds(0.5);
        //drive.updatePoseEstimate();

        // 1st sample: drive to and turn on intake
        Pose2d toFirstSample = new Pose2d(12,16, Math.toRadians(135));
        Action DriveToFirstSample = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toFirstSample.position, toFirstSample.heading, new TranslationalVelConstraint(20.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToFirstSample,
                new SequentialAction(
                        new SleepAction(1),
                        new InstantAction(() -> extras.elevatorDown()),
                        new SleepAction(1),
                        new InstantAction(() -> extras.armExtend()),
                        new SleepAction(0.5),
                        new InstantAction(() -> extras.intakeIn())
                )
        ));

        safeWaitSeconds(1);
        //drive.updatePoseEstimate();

        // 1st sample: pick up the first sample by driving into it
        Pose2d firstsample = new Pose2d(8,19, Math.toRadians(135));
        Action FirstSampleAction = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(firstsample.position, new TranslationalVelConstraint(10.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                FirstSampleAction,
                new SequentialAction(
                        new SleepAction(3),
                        new InstantAction(() -> extras.intakeOff()),
                        new SleepAction(1.0)
                )
        ));

        //1st sample: move to the basket and put it in the basket
        Pose2d toPole2 = new Pose2d(-20.5,6, Math.toRadians(45));
        Pose2d nearPole2 = new Pose2d(-20.5,6, Math.toRadians(45));
        Action DriveToPole = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(nearPole2.position, nearPole2.heading, baseVelConstraint)
                .strafeToLinearHeading(toPole2.position, toPole2.heading, baseVelConstraint, new ProfileAccelConstraint(-40,40))
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToPole,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armRetract()),
                        new SleepAction(1.6),
                        new InstantAction(() -> extras.intakeOut()),
                        new SleepAction(3),
                        new InstantAction(() -> extras.elevatorHighBasket()),
                        new SleepAction(1),
                        new InstantAction(() -> extras.sampleDump()),
                        new SleepAction(1),
                        new InstantAction(() -> extras.samplePickup()),
                        new SleepAction(0.5)
                )));

        safeWaitSeconds(5);

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


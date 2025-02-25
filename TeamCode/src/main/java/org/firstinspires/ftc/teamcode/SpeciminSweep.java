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
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;

@Config
//@Disabled
@Autonomous(group = "a")

public class SpeciminSweep extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
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

        double initialRotation = 180;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));
        Pose2d toSubmursible = new Pose2d(29,10,Math.toRadians(180));
        Pose2d backUpFromSubmursible = new Pose2d(19,-11,Math.toRadians(315));
        Pose2d lineUpForSweep1 = new Pose2d(20,-14,Math.toRadians(-45));
        Pose2d lineUpForSweep2 = new Pose2d(20,-24,Math.toRadians(-45));
        Pose2d lineUpForSweep3 = new Pose2d(20,-35,Math.toRadians(-45));
        Pose2d sweep3 = new Pose2d(10,-28,Math.toRadians(-110));
        Pose2d lineUpForWallSlide = new Pose2d(0,-8,Math.toRadians(-270));
        Pose2d wallSlide = new Pose2d(-5,-49,Math.toRadians(90));
        Pose2d lineUpForWallSlide2 = new Pose2d(0,-8,Math.toRadians(160));
        Pose2d wallSlide2 = new Pose2d(-5,-20,Math.toRadians(170));
        Pose2d toSubmursible2 = new Pose2d(29,60,Math.toRadians(180));
        Pose2d toSubmursible3 = new Pose2d(29,59,Math.toRadians(270));
        Pose2d toSubmursible4 = new Pose2d(29,58,Math.toRadians(270));
        Pose2d toSubmursible5 = new Pose2d(29,57,Math.toRadians(270));
        Pose2d park = new Pose2d(-10,-2,Math.toRadians(275));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        // hang pre-loaded sample in the high chamber
        Action DriveToNearSubmursibleAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toSubmursible.position, toSubmursible.heading, new TranslationalVelConstraint(55.0))
                .build();

        Action DriveToNearSubmursibleAction2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toSubmursible2.position, toSubmursible2.heading, new TranslationalVelConstraint(50.0))
                // move to submersible
                .build();

        //sleep(500);

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", Math.toDegrees(drive.pose.heading.toDouble()));
        ppPos = drive.odo.getPosition();
        telemetry.addData("x", ppPos.getX(DistanceUnit.INCH));
        telemetry.addData("y", ppPos.getY(DistanceUnit.INCH));
        telemetry.addData("odo heading", ppPos.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {
            safeWaitSeconds(0.010);
        }

        Actions.runBlocking(new ParallelAction(
                DriveToNearSubmursibleAction,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new InstantAction(() -> extras.elevatorHighChamber()),
                        new SleepAction(1.2),
                        new InstantAction(() -> extras.elevatorDown()),
                        new InstantAction(() -> extras.armExtend()),
                        new SleepAction(0.0)
                )
        ));

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(backUpFromSubmursible.position, backUpFromSubmursible.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(lineUpForSweep1.position, lineUpForSweep1.heading)
                        .turnTo(Math.toRadians(-110))
                        .turnTo(Math.toRadians(-45))
                        .strafeToLinearHeading(lineUpForSweep2.position, lineUpForSweep2.heading)
                        .turnTo(Math.toRadians(-110))
                        .turnTo(Math.toRadians(-45))
                        .strafeToLinearHeading(lineUpForSweep3.position, lineUpForSweep3.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(sweep3.position, sweep3.heading, new TranslationalVelConstraint(50.0))
                        .build());

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> extras.armVertical()),
                new SleepAction(0.1),
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(lineUpForWallSlide, lineUpForWallSlide.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(wallSlide.position,wallSlide.heading)
                        .build()
            )
        );

        drive.odo.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES, -90));
        drive.pose = new Pose2d(0,0,Math.toRadians(-90));

        Actions.runBlocking(new ParallelAction(
                DriveToNearSubmursibleAction2,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.elevatorHighChamber()),
                        new SleepAction(2.3),
                        new InstantAction(() -> extras.elevatorDown()),
                        new SleepAction(0.15)
                )
        ));

        safeWaitSeconds(30);


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
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}
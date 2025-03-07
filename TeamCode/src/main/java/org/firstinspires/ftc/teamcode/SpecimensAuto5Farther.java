package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@Autonomous(group = "a")


public class SpecimensAuto5Farther extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(initialRotation));
        Pose2d toSubmursible = new Pose2d(-13, 31, Math.toRadians(270));
        Pose2d backUpFromSubmursible = new Pose2d(20, 26, Math.toRadians(270));
        Pose2d forward1 = new Pose2d(20, 38, Math.toRadians(270));
        Pose2d slideOver1 = new Pose2d(34, 50, Math.toRadians(270));
        Pose2d sweep = new Pose2d(33, 18, Math.toRadians(270));
        Pose2d backTo1 = new Pose2d(32, 45, Math.toRadians(270));
        Pose2d slideOver2 = new Pose2d(50, 45, Math.toRadians(270));
        Pose2d sweep2 = new Pose2d(51, 16, Math.toRadians(270));
        Pose2d backTo2 = new Pose2d(45, 48, Math.toRadians(270));
        Pose2d slideOver3 = new Pose2d(63, 48, Math.toRadians(270));
        Pose2d sweep3 = new Pose2d(70, 10, Math.toRadians(270));
        Pose2d sweep3a = new Pose2d(65, 15, Math.toRadians(270));
        Pose2d lineUpForWallSlide = new Pose2d(55, -15, Math.toRadians(180));
        Pose2d wallSlide = new Pose2d(72, -15, Math.toRadians(180));
        Pose2d lineUpForWallSlide2 = new Pose2d(-20, -7, Math.toRadians(180));
        Pose2d wallSlide2 = new Pose2d(1, -7, Math.toRadians(180));
        Pose2d infrontOfSubmursible = new Pose2d(-61, 35, Math.toRadians(270));
        Pose2d toSubmursible2 = new Pose2d(-61, 39, Math.toRadians(270));
        Pose2d toSubmursible3 = new Pose2d(-65, 39, Math.toRadians(270));
        Pose2d toSubmursible4 = new Pose2d(-67, 39, Math.toRadians(270));
        Pose2d toSubmursible5 = new Pose2d(-69, 39, Math.toRadians(270));
        Pose2d park = new Pose2d(-15, 20, Math.toRadians(315));

        Pose2D ppPos;

        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        extras.saveAutoStartRotation(Math.toRadians(45) + Math.toRadians(initialRotation));

        boolean manualLiftStop = false;

        extras.initArm();
        extras.initElevator();
        extras.tailUp();
        extras.armRetract();

        while (!gamepad2.dpad_left) {
            sleep(100);
            telemetry.addLine("Align robot to wall");
            telemetry.addLine("GP2 DPad Left to exit");
            telemetry.update();

            if(gamepad2.left_bumper) {
                extras.liftRetractManual();
                manualLiftStop = true;
            }
            else if(gamepad2.left_trigger > 0) {
                extras.liftExtendManual();
                manualLiftStop = true;
            }
            else
            {
                if(manualLiftStop == true)
                {
                    extras.liftStop();
                    manualLiftStop = false;
                }
            }
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        // hang pre-loaded sample in the high chamber
        TrajectoryActionBuilder driveToNearSubmursible = drive.actionBuilder(initPose)
                .strafeToLinearHeading(toSubmursible.position, toSubmursible.heading, new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-50, 50));

        Action DriveToNearSubmursibleAction = driveToNearSubmursible.build();

        Action SweepAction = driveToNearSubmursible.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(backUpFromSubmursible, Math.toRadians(90), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                .splineToSplineHeading(forward1, Math.toRadians(90), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                .splineToLinearHeading(slideOver1, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                .splineToLinearHeading(sweep, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                .splineToLinearHeading(backTo1, Math.toRadians(0), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                .splineToLinearHeading(slideOver2, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                .splineToLinearHeading(sweep2, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                .splineToLinearHeading(backTo2, Math.toRadians(0), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                .splineToLinearHeading(slideOver3, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                .splineToLinearHeading(sweep3, Math.toRadians(270), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                //.setTangent(Math.toRadians(135))
                .splineToLinearHeading(sweep3a, Math.toRadians(0), new TranslationalVelConstraint(45.0), new ProfileAccelConstraint(-70, 70))
                .build();

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        ppPos = drive.odo.getPosition();
        telemetry.update();

        try {

            while (!isStopRequested() && !opModeIsActive()) {
                safeWaitSeconds(0.01);
            }

            Actions.runBlocking(new ParallelAction(
                    DriveToNearSubmursibleAction,
                    new SequentialAction(
                            new SleepAction(0),
                            new InstantAction(() -> extras.armVertical()),
                            new InstantAction(() -> extras.elevatorHighChamber()),
                            new SleepAction(1.2),
                            new InstantAction(() -> extras.elevatorHalf())
                    )
            ));

            Actions.runBlocking(new ParallelAction(
                    SweepAction,
                    new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> extras.elevatorDown())
                    )
            ));

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineToLinearHeading(lineUpForWallSlide, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                            .splineToLinearHeading(wallSlide, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                            .build());

            drive.pose = new Pose2d(0, 0, Math.toRadians(180));

            Action DriveToNearSubmursibleAction2 = drive.actionBuilder(drive.pose)
                    .splineToLinearHeading(toSubmursible2,  Math.toRadians(90), new TranslationalVelConstraint(50.0))
                    // move to submersible
                    .build();

            Actions.runBlocking(new ParallelAction(
                    DriveToNearSubmursibleAction2,
                    new SequentialAction(
                            new SleepAction(0),
                            new InstantAction(() -> extras.armVertical()),
                            new InstantAction(() -> extras.elevatorHighChamber()),
                            new SleepAction(2.1),
                            new InstantAction(() -> extras.elevatorHalf())
                    )
            ));

            Action ElevatorHalfAction = drive.actionBuilder(drive.pose)
                    .setTangent(Math.toRadians(-45))
                    .splineToLinearHeading(lineUpForWallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                    .splineToSplineHeading(wallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                    .build();

            Actions.runBlocking(new ParallelAction(
                    ElevatorHalfAction,
                    new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> extras.elevatorDown())

                    )
            ));

            drive.pose = new Pose2d(0, 0, Math.toRadians(180));

            Action DriveToNearSubmursibleAction3 = drive.actionBuilder(drive.pose)
                    .splineToLinearHeading(toSubmursible3,  Math.toRadians(90), new TranslationalVelConstraint(50.0))
                    // move to submersible
                    .build();

            Actions.runBlocking(new ParallelAction(
                    DriveToNearSubmursibleAction3,
                    new SequentialAction(
                            new SleepAction(0),
                            new InstantAction(() -> extras.armVertical()),
                            new InstantAction(() -> extras.elevatorHighChamber()),
                            new SleepAction(2.1),
                            new InstantAction(() -> extras.elevatorHalf())
                    )
            ));

            Action ElevatorHalfAction2 = drive.actionBuilder(drive.pose)
                            .setTangent(Math.toRadians(-45))
                            .splineToLinearHeading(lineUpForWallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                            .splineToSplineHeading(wallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                            .build();

            Actions.runBlocking(new ParallelAction(
                    ElevatorHalfAction2,
                    new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> extras.elevatorDown())

                    )
            ));

            drive.pose = new Pose2d(0, 0, Math.toRadians(180));

            Action DriveToNearSubmursibleAction4 = drive.actionBuilder(drive.pose)
                    .splineToLinearHeading(toSubmursible4,  Math.toRadians(90), new TranslationalVelConstraint(50.0))
                    // move to submersible
                    .build();

            Actions.runBlocking(new ParallelAction(
                    DriveToNearSubmursibleAction4,
                    new SequentialAction(
                            new SleepAction(0),
                            new InstantAction(() -> extras.armVertical()),
                            new InstantAction(() -> extras.elevatorHighChamber()),
                            new SleepAction(2.1),
                            new InstantAction(() -> extras.elevatorHalf())
                    )
            ));


            Action ElevatorHalfAction3 = drive.actionBuilder(drive.pose)
                    .setTangent(Math.toRadians(-45))
                    .splineToLinearHeading(lineUpForWallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                    .splineToSplineHeading(wallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                    .build();

            Actions.runBlocking(new ParallelAction(
                    ElevatorHalfAction3,
                    new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> extras.elevatorDown())

                    )
            ));

            drive.pose = new Pose2d(0, 0, Math.toRadians(180));

            Action DriveToNearSubmursibleAction5 = drive.actionBuilder(drive.pose)
                    .splineToLinearHeading(toSubmursible5,  Math.toRadians(90), new TranslationalVelConstraint(50.0))
                    // move to submersible
                    .build();

            Actions.runBlocking(new ParallelAction(
                    DriveToNearSubmursibleAction5,
                    new SequentialAction(
                            new SleepAction(0),
                            new InstantAction(() -> extras.armVertical()),
                            new InstantAction(() -> extras.elevatorHighChamber()),
                            new SleepAction(2.1),
                            new InstantAction(() -> extras.elevatorHalf())
                    )
            ));

            Action parkAction = drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(park.position, park.heading, new TranslationalVelConstraint(50.0))
                    // move to submersible
                    .build();

            Actions.runBlocking(new ParallelAction(
                    parkAction,
                    new SequentialAction(
                            new SleepAction(0.2),
                            new InstantAction(() -> extras.elevatorDown()),
                            new InstantAction(() -> extras.armExtend())
                    )
            ));

            // Save the ending location
            //extras.saveAutoStartRotation(drive.odo.getHeading() + Math.toRadians(initialRotation) - PI / 2);
            extras.saveAutoStartRotation(Math.toRadians(45) + Math.toRadians(initialRotation));

        }
        catch (Exception e)
        {
            //double heading = drive.odo.getHeading() + Math.toRadians(initialRotation) - PI / 2;
            double heading = Math.toRadians(45) + Math.toRadians(initialRotation);
            extras.saveAutoStartRotation(heading);
            telemetry.addData("heading", heading);
            telemetry.update();
        }
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


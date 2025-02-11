package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(group = "a")

public class SpecimensAutoPickup3 extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));
        Pose2d toSubmursible = new Pose2d(-9,30,Math.toRadians(270));
        Pose2d backUpFromSubmursible = new Pose2d(13,17,Math.toRadians(405));
        Pose2d pickUpFirst = new Pose2d(17,21,Math.toRadians(405));
        Pose2d secondSample = new Pose2d(23,16.5,Math.toRadians(405));
        Pose2d pickUpSecond = new Pose2d(26,19.5,Math.toRadians(405));
        Pose2d backUp = new Pose2d(25,18,Math.toRadians(405));
        Pose2d thirdSample = new Pose2d(31,16,Math.toRadians(405));
        Pose2d pickUpThird = new Pose2d(35,20,Math.toRadians(405));
        Pose2d backUp2 = new Pose2d(25,17,Math.toRadians(405));

        Pose2d lineUpForWallSlide = new Pose2d(31,-5,Math.toRadians(180));
        Pose2d wallSlide = new Pose2d(52,-3,Math.toRadians(180));
        Pose2d lineUpForWallSlide2 = new Pose2d(-10,-20,Math.toRadians(180));
        Pose2d wallSlide2 = new Pose2d(-3,-20,Math.toRadians(180));
        Pose2d toSubmursible2 = new Pose2d(-67,34,Math.toRadians(265));
        Pose2d toSubmursible3 = new Pose2d(-67,34,Math.toRadians(265));
        Pose2d toSubmursible4 = new Pose2d(-67,34,Math.toRadians(265));
        Pose2d toSubmursible5 = new Pose2d(-67,34,Math.toRadians(265));
        Pose2d park = new Pose2d(-15,-2,Math.toRadians(265));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        // hang pre-loaded sample in the high chamber
        Action DriveToNearSubmursibleAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toSubmursible.position, toSubmursible.heading, new TranslationalVelConstraint(45.0))
                // move to submersible
                //.strafeToLinearHeading(new Vector2d(-15,29), Rotation2d.exp(270), new TranslationalVelConstraint(15.0))
                .build();

        Action DriveToNearSubmursibleAction2 = drive.actionBuilder(new Pose2d(0,0,Math.toRadians(180)))
                .strafeToLinearHeading(toSubmursible2.position, toSubmursible2.heading, new TranslationalVelConstraint(55.0))
                // move to submersible
                .build();

        Action DriveToNearSubmursibleAction3 = drive.actionBuilder(new Pose2d(0,0,Math.toRadians(180)))
                .strafeToLinearHeading(toSubmursible3.position, toSubmursible3.heading, new TranslationalVelConstraint(55.0))
                // move to submersible
                .build();

        Action DriveToNearSubmursibleAction4 = drive.actionBuilder(new Pose2d(0,0,Math.toRadians(180)))
                .strafeToLinearHeading(toSubmursible4.position, toSubmursible4.heading, new TranslationalVelConstraint(55.0))
                // move to submersible
                .build();

        Action DriveToNearSubmursibleAction5 = drive.actionBuilder(new Pose2d(0,0,Math.toRadians(180)))
                .strafeToLinearHeading(toSubmursible5.position, toSubmursible5.heading, new TranslationalVelConstraint(55.0))
                // move to submersible
                .build();




        drive.PARAMS.axialGain = 8.5;
        drive.PARAMS.lateralGain = 8.0;
        drive.PARAMS.headingGain = 8.0;
        drive.PARAMS.axialVelGain = 1.0;
        drive.PARAMS.lateralVelGain = 0.0;
        drive.PARAMS.headingVelGain = 0.0;

        extras.initArm();
        extras.tailUp();

        boolean manualLiftStop = false;

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        telemetry.update();

        /*
        while (!gamepad2.dpad_left)
        {
            sleep(100);
            telemetry.addLine("Adjust Hanging arm with left bumpers");
            telemetry.addLine("DPad Left to exit");
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

         */

        while (!isStopRequested() && !opModeIsActive())
        {
            safeWaitSeconds(0.01);
        }



        Actions.runBlocking(new ParallelAction(
                DriveToNearSubmursibleAction,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new InstantAction(() -> extras.elevatorHighChamber()),
                        new SleepAction(1.5),
                        new InstantAction(() -> extras.elevatorDown())
                )
        ));

        safeWaitSeconds(0.15);

        Action FirstSampleAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(backUpFromSubmursible.position, backUpFromSubmursible.heading, new TranslationalVelConstraint(20.0))
                .build();
        Actions.runBlocking(new ParallelAction(
                FirstSampleAction,
                new SequentialAction(
                        new InstantAction(() -> extras.armExtend()),
                        new InstantAction(() -> extras.intakeIn()),
                        new SleepAction(2.0)
                )
        ));

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(pickUpFirst.position, new TranslationalVelConstraint(20.0))
                        .build());

        safeWaitSeconds(0.5);

        Action DropFirstSampleAction = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(315))

                .build();
        Actions.runBlocking(new ParallelAction(
                DropFirstSampleAction,
                new SequentialAction(
                        new SleepAction(0.3),
                        new InstantAction(() -> extras.intakeOut()),
                        new SleepAction(0.5)
                )
        ));

        Action SecondSampleAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(secondSample.position, secondSample.heading, new TranslationalVelConstraint(20.0))
                .build();
        Actions.runBlocking(new ParallelAction(
                SecondSampleAction,
                new SequentialAction(
                        new InstantAction(() -> extras.intakeIn()),
                        new SleepAction(2.0)
                )
        ));

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(pickUpSecond.position, new TranslationalVelConstraint(20.0))
                        .build());

        safeWaitSeconds(1.0);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(backUp.position, backUp2.heading, new TranslationalVelConstraint(20.0))
                        .build());
        safeWaitSeconds(1.0);

        Action DropSecondSampleAction = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(315))
                .build();

        Actions.runBlocking(new ParallelAction(
                DropSecondSampleAction,
                new SequentialAction(
                        new SleepAction(0.3),
                        new InstantAction(() -> extras.intakeOut()),
                        new SleepAction(0.5)
                )
        ));

        Action ThirdSampleAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(thirdSample.position, thirdSample.heading, new TranslationalVelConstraint(20.0))
                .build();
        Actions.runBlocking(new ParallelAction(
                ThirdSampleAction,
                new SequentialAction(
                        new InstantAction(() -> extras.intakeIn()),
                        new SleepAction(2.0)
                )
        ));

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(pickUpThird.position, new TranslationalVelConstraint(20.0))
                        .build());

        safeWaitSeconds(1.0);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(backUp2.position, backUp2.heading, new TranslationalVelConstraint(20.0))
                        .build());



        safeWaitSeconds(10.0);

        Action DropThirdSampleAction = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(315))
                .build();

        Actions.runBlocking(new ParallelAction(
                DropThirdSampleAction,
                new SequentialAction(
                        new SleepAction(1.0),
                        new InstantAction(() -> extras.intakeOut()),
                        new SleepAction(0.5)
                )
        ));


        /*

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(lineUpForWallSlide.position, lineUpForWallSlide.heading, new TranslationalVelConstraint(55.0))
                        .strafeToLinearHeading(wallSlide.position, wallSlide.heading, new TranslationalVelConstraint(55.0))
                        .build());

        drive.odo.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES, 180-initialRotation));
        drive.pose = new Pose2d(0,0,Math.toRadians(180));

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
                        //.strafeToLinearHeading(lineUpForWallSlide2.position, lineUpForWallSlide2.heading, new TranslationalVelConstraint(55.0))
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(lineUpForWallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                        .splineToSplineHeading(wallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                        .build());


        drive.odo.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES, 180-initialRotation));
        drive.pose = new Pose2d(0,0,Math.toRadians(180));

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
                        //.strafeToLinearHeading(lineUpForWallSlide2.position, lineUpForWallSlide2.heading, new TranslationalVelConstraint(55.0))
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(lineUpForWallSlide2, Math.toRadians(0), new TranslationalVelConstraint(50.0))
                        .splineToSplineHeading(wallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                        .build());

        drive.odo.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES, 180-initialRotation));
        drive.pose = new Pose2d(0,0,Math.toRadians(180));

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
                        //.strafeToLinearHeading(lineUpForWallSlide2.position, lineUpForWallSlide2.heading, new TranslationalVelConstraint(55.0))
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(lineUpForWallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                        .splineToSplineHeading(wallSlide2, Math.toRadians(0), new TranslationalVelConstraint(55.0))
                        .build());

        drive.odo.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES, 180-initialRotation));
        drive.pose = new Pose2d(0,0,Math.toRadians(180));

        Actions.runBlocking(new ParallelAction(
                DriveToNearSubmursibleAction5,
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
                .strafeToLinearHeading(park.position, park.heading, new TranslationalVelConstraint(55.0))
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


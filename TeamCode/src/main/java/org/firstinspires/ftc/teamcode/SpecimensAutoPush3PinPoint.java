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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@Autonomous(group = "a")
@Disabled

public class SpecimensAutoPush3PinPoint extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));
        Pose2d toSubmursible = new Pose2d(-10,29,Math.toRadians(270));
        Pose2d backUpFromSubmursible = new Pose2d(18,25,Math.toRadians(270));
        //Pose2d backUpFromSubmursible2 = new Pose2d(18,29,Math.toRadians(270));
        //Pose2d lineUpForSweep = new Pose2d(30,50,Math.toRadians(270));
        Pose2d forward1 = new Pose2d(18,38,Math.toRadians(270));
        Pose2d slideOver1 = new Pose2d(36,51,Math.toRadians(270));
        Pose2d sweep = new Pose2d(31,15,Math.toRadians(270));
        Pose2d backTo1 = new Pose2d(31,38,Math.toRadians(270));
        Pose2d slideOver2 = new Pose2d(43,40,Math.toRadians(270));
        Pose2d sweep2 = new Pose2d(42,13.5,Math.toRadians(270));
        Pose2d backTo2 = new Pose2d(40,38,Math.toRadians(270));
        Pose2d slideOver3 = new Pose2d(54.5,42,Math.toRadians(270));
        Pose2d sweep3 = new Pose2d(53,9,Math.toRadians(270));
        Pose2d sweep3a = new Pose2d(45,16,Math.toRadians(270));
        Pose2d lineUpForWallSlide = new Pose2d(31,-5,Math.toRadians(180));
        Pose2d wallSlide = new Pose2d(52,-5,Math.toRadians(180));
        Pose2d lineUpForWallSlide2 = new Pose2d(-10,-20,Math.toRadians(160));
        Pose2d wallSlide2 = new Pose2d(-1,-20,Math.toRadians(170));
        Pose2d toSubmursible2 = new Pose2d(-67,34,Math.toRadians(270));
        Pose2d toSubmursible3 = new Pose2d(-67,34,Math.toRadians(270));
        Pose2d toSubmursible4 = new Pose2d(-67,34,Math.toRadians(270));
        Pose2d toSubmursible5 = new Pose2d(-67,34,Math.toRadians(270));
        Pose2d park = new Pose2d(-10,-2,Math.toRadians(275));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        // hang pre-loaded sample in the high chamber
        Action DriveToNearSubmursibleAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toSubmursible.position, toSubmursible.heading, new TranslationalVelConstraint(55.0), new ProfileAccelConstraint(-50,50))
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
        drive.PARAMS.headingGain = 2.0;
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
                        new SleepAction(1.2),
                        new InstantAction(() -> extras.elevatorDown())
                )
        ));

        safeWaitSeconds(0.0);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(backUpFromSubmursible, Math.toRadians(90), new TranslationalVelConstraint(50.0))
                        //.splineToLinearHeading(backUpFromSubmursible2, Math.toRadians(90), new TranslationalVelConstraint(55.0))
                        .splineToSplineHeading(forward1, Math.toRadians(90), new TranslationalVelConstraint(50.0))
                        .splineToLinearHeading(slideOver1, Math.toRadians(180), new TranslationalVelConstraint(50.0))
                        .splineToLinearHeading(sweep, sweep.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(backTo1.position, backTo1.heading, new TranslationalVelConstraint(50.0))
                        .splineToLinearHeading(slideOver2, slideOver2.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(sweep2.position, sweep2.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(backTo2.position, backTo2.heading, new TranslationalVelConstraint(50.0))
                        .splineToLinearHeading(slideOver3, slideOver3.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(sweep3.position, sweep3.heading, new TranslationalVelConstraint(50.0))
                        .setTangent(Math.toRadians(135))
                        .strafeToLinearHeading(sweep3a.position, sweep3a.heading, new TranslationalVelConstraint(50.0))
                        .build());

        drive.PARAMS.axialGain = 8.5;
        drive.PARAMS.lateralGain = 2.0;
        drive.PARAMS.headingGain = 1.0;
        drive.PARAMS.axialVelGain = 0.0;
        drive.PARAMS.lateralVelGain = 0.0;
        drive.PARAMS.headingVelGain = 0.0;

        //safeWaitSeconds(0.4);

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
        safeWaitSeconds(5);

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


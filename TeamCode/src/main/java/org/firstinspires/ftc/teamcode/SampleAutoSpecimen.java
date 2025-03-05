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
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(group = "a")
//@Disabled

public class SampleAutoSpecimen extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));
        Pose2d toSubmursible = new Pose2d(35,30,Math.toRadians(270));
        Pose2d backUpFromSubmursible = new Pose2d(5,26,Math.toRadians(270));
        Pose2d slideOver1 = new Pose2d(-10,53,Math.toRadians(270));
        Pose2d sweep = new Pose2d(-10,7,Math.toRadians(270));
        Pose2d backTo1 = new Pose2d(-10,52,Math.toRadians(270));
        Pose2d slideOver2 = new Pose2d(-20,52,Math.toRadians(270));
        Pose2d sweep2 = new Pose2d(-20,10,Math.toRadians(270));
        Pose2d backTo2 = new Pose2d(-20,52,Math.toRadians(270));
        Pose2d slideOver3 = new Pose2d(-30,52,Math.toRadians(270));
        Pose2d sweep3 = new Pose2d(-30,16,Math.toRadians(270));
        Pose2d lineUpForPark = new Pose2d(0,57,Math.toRadians(360));
        Pose2d park = new Pose2d(12,57,Math.toRadians(360));


        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);


        extras.initArm();
        extras.initElevator();
        extras.tailUp();
        extras.armRetract();

        while (!gamepad2.dpad_left) {
            sleep(100);
            telemetry.addLine("Align robot to wall");
            telemetry.addLine("GP2 DPad Left to exit");
            telemetry.update();
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        drive.PARAMS.headingGain = 7.0;


        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading",Math.toDegrees(drive.pose.heading.toDouble()));
        drive.odo.update();
        telemetry.addData("ODO heading", drive.odo.getHeading());
        telemetry.update();

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
                //.strafeToLinearHeading(new Vector2d(25,29), Rotation2d.exp(270), new TranslationalVelConstraint(15.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToNearSubmursibleAction,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new InstantAction(() -> extras.elevatorHighChamber()),
                        new SleepAction(1.6),
                        new InstantAction(() -> extras.elevatorDown())
                )
        ));

        safeWaitSeconds(0.15);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(backUpFromSubmursible.position, backUpFromSubmursible.heading, new TranslationalVelConstraint(25.0))
                        //.strafeToLinearHeading(lineUpForSweep.position, lineUpForSweep.heading, new TranslationalVelConstraint(50.0))
                        .strafeToLinearHeading(slideOver1.position, slideOver1.heading, new TranslationalVelConstraint(25.0))
                        .strafeToLinearHeading(sweep.position, sweep.heading, new TranslationalVelConstraint(25.0))
                        .strafeToLinearHeading(backTo1.position, backTo1.heading, new TranslationalVelConstraint(25.0))
                        .strafeToLinearHeading(slideOver2.position, slideOver2.heading, new TranslationalVelConstraint(25.0))
                        .strafeToLinearHeading(sweep2.position, sweep2.heading, new TranslationalVelConstraint(25.0))
                        .strafeToLinearHeading(backTo2.position, backTo2.heading, new TranslationalVelConstraint(25.0))
                        .strafeToLinearHeading(slideOver3.position, slideOver3.heading, new TranslationalVelConstraint(25.0))
                        .strafeToLinearHeading(sweep3.position, sweep3.heading, new TranslationalVelConstraint(25.0))
                        .build());


        Action LineUpForParkAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(lineUpForPark.position, lineUpForPark.heading, new TranslationalVelConstraint(50.0), new ProfileAccelConstraint(-50,50))
                // move to submersible
                //.strafeToLinearHeading(new Vector2d(25,29), Rotation2d.exp(270), new TranslationalVelConstraint(15.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                LineUpForParkAction,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armPark())

                )
        ));

        extras.arm.setPower(0);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(park.position, park.heading, new TranslationalVelConstraint(3.0))
                        .build());

        safeWaitSeconds(0.5);


        safeWaitSeconds(10);

        extras.arm.setPower(0);

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
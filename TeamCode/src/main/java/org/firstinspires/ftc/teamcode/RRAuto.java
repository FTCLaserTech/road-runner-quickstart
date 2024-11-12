package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

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
//@Disabled

public class RRAuto extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        Pose2d purplePixelDropoff = new Pose2d(0,0,0);
        Pose2d straightenRobot = new Pose2d(0,0,0);
        Pose2d park = new Pose2d(0,0,0);
        Pose2d forwardPark = new Pose2d(0,0,0);
        Pose2d leftPark = new Pose2d(0,0,0);
        Pose2d prePaint = new Pose2d(0,0,0);
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(90));
        Pose2d paint = new Pose2d(0,0,0);


        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this, ExtraOpModeFunctions.FieldSide.RED);

        //sleep(500);
        //extras.initElevator();

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        telemetry.update();

        ExtraOpModeFunctions.Signal signal = ExtraOpModeFunctions.Signal.LEFT;

        while (!isStopRequested() && !opModeIsActive()) {
            //signal = extras.telemetryTfod(ExtraOpModeFunctions.AutoStart.RR);
            telemetry.addData("Position", signal);
            telemetry.update();
            sleep(20);
        }

        switch(signal)
        {
            case LEFT:
                // move to LEFT column of parking tiles
                purplePixelDropoff = new Pose2d(-12.3,23.19,Math.toRadians(135));
                straightenRobot = new Pose2d(-2,12,Math.toRadians(90));
                prePaint = new Pose2d(30,31,Math.toRadians(0));
                paint = new Pose2d(35,31,Math.toRadians(0));
                break;

            case MIDDLE:
                // move to MIDDLE column of parking tiles
                purplePixelDropoff = new Pose2d(-1,28,Math.toRadians(90));
                straightenRobot = new Pose2d(-1,12,Math.toRadians(90));
                prePaint = new Pose2d(30,24,Math.toRadians(0));
                paint = new Pose2d(35,24,Math.toRadians(0));
                break;

            case RIGHT:
                // move to RIGHT colum of parking tiles
                purplePixelDropoff = new Pose2d(6,23.2,Math.toRadians(70));
                straightenRobot = new Pose2d(2,12,Math.toRadians(90));
                prePaint = new Pose2d(30,16,Math.toRadians(0));
                paint = new Pose2d(35.3,16,Math.toRadians(0));
                break;

        }

        park = new Pose2d(30,-2,Math.toRadians(90));


        telemetry.addData("Place Pixel", 0);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(purplePixelDropoff.position, purplePixelDropoff.heading, new TranslationalVelConstraint(20.0))
                        .build());

        sleep(200);
        telemetry.addData("Back Up", 0);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(straightenRobot.position, straightenRobot.heading, new TranslationalVelConstraint(20.0))
                        //.splineToLinearHeading(straightenRobot,Math.toRadians(0))
                        .build());

        sleep(200);
        telemetry.addData("Painting", 0);
        telemetry.update();

        Action DriveToNearBackDropAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(prePaint.position, prePaint.heading, new TranslationalVelConstraint(20.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToNearBackDropAction,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.elevatorOne(1)),
                        new SleepAction(2)
                )
        ));

        Action DriveToBackDropAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(paint.position, paint.heading, new TranslationalVelConstraint(10.0))
                .build();

        Actions.runBlocking(new SequentialAction(
                DriveToBackDropAction,
                new SleepAction(1)
            )
        );

        sleep(500);
        telemetry.addData("All Done", 0);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(prePaint.position, prePaint.heading)
                        .build());

        Action DriveToParkAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(park.position, park.heading)
                .build();

        Actions.runBlocking(new ParallelAction(
                        DriveToParkAction,
                        new InstantAction(() -> extras.elevatorCollect())
                )
        );
    }


    public void safeWaitSeconds(double time)
    {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }


}
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(group = "a")
@Disabled

public class RRAuto extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Pose2d purplePixelDropoff = new Pose2d(0,0,0);
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(90));
        Pose2d left = new Pose2d(0,0,0);
        Pose2d dropoff = new Pose2d(0,0,0);
        Pose2d mid = new Pose2d(0,0,0);
        Pose2d right = new Pose2d(0,0,0);

        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this, ExtraOpModeFunctions.FieldSide.RED);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        //sleep(500);
        //extras.initElevator();

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        telemetry.update();


        while (!isStopRequested() && !opModeIsActive()) {


            left = new Pose2d(35, 31, Math.toRadians(0));
            dropoff = new Pose2d(35, 31, Math.toRadians(0));

            //Left sample
            Action DriveToNearBackDropAction = drive.actionBuilder(drive.pose)
                    .strafeToLinearHeading(left.position, left.heading, new TranslationalVelConstraint(20.0))
                    .build();

            Actions.runBlocking(new ParallelAction(
                    DriveToNearBackDropAction,
                    new SequentialAction(
                            new SleepAction(0),
                            new InstantAction(() -> extras.armExtend()),
                            new InstantAction(() -> extras.intakeIn()),
                            new SleepAction(2)
                    )
            ));
            //Drop off

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(dropoff.position, dropoff.heading, new TranslationalVelConstraint(20.0))
                            .build());

            //Mid sample

            //Drop off

            //Right sample

            //Drop off

            //Grab Specimen 1

            //Drop off specimen 1

            //Grab Specimen 2

            //Drop off specimen 2

            //Grab Specimen 3

            //Drop off specimen 3

            //Grab Specimen 4

            //Drop off specimen 4


        }
    }


    public void safeWaitSeconds(double time)
    {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }


}


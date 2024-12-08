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

public class SpecimensAuto extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        double initialRotation = 270;
        //Pose2d purplePixelDropoff = new Pose2d(0,0,0);
        Pose2d initPose = new Pose2d(0,0,Math.toRadians(initialRotation));
        Pose2d toSubmursible = new Pose2d(-15,29,Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        //sleep(500);
        //extras.initElevator();

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {
            sleep(10);
        }

        // hang pre-loaded sample in the high chamber
        Action DriveToNearSubmursibleAction = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toSubmursible.position, toSubmursible.heading, new TranslationalVelConstraint(15.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToNearSubmursibleAction,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new InstantAction(() -> extras.elevatorHighChamber()),
                        new SleepAction(5),
                        new InstantAction(() -> extras.elevatorSpecimanGrab())
                )
        ));

        sleep(5000);

        // Move 3 samples from the field into the drop off

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


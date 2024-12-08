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

public class SamplesAuto extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Pose2d initPose = new Pose2d(0,0, Math.toRadians(90));
        Pose2d toPole = new Pose2d(-15,10, Math.toRadians(45));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        //sleep(500);
        extras.initArm();

        telemetry.addLine("Initialized");
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading", drive.pose.heading);
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {
            safeWaitSeconds(0.010);
            //sleep(10);
        }

        // put the preloaded sample in the high basket
        Action DriveToBasketPole = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(toPole.position, toPole.heading, new TranslationalVelConstraint(15.0))
                .build();

        Actions.runBlocking(new ParallelAction(
                DriveToBasketPole,
                new SequentialAction(
                        new SleepAction(0),
                        new InstantAction(() -> extras.armVertical()),
                        new InstantAction(() -> extras.elevatorHighBasket()),
                        new SleepAction(5),
                        new InstantAction(() -> extras.sampleDump())
                )
        ));

        sleep(5000);

        //pick up the 1st sample on the field and put it in the high basket

        //pick up the 2nd sample on the field and put it in the high basket

        //pick up the 3rd sample on the field and put it in the high basket

        //Park

        // Save the ending location
        extras.saveAutoStartRotation(drive.odo.getHeading());
    }

    public void safeWaitSeconds(double time)
    {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}


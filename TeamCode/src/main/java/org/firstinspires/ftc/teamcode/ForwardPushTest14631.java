package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(group = "A")
public class ForwardPushTest14631 extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0), this);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this, ExtraOpModeFunctions.FieldSide.BLUE);

        waitForStart();

        while (!isStopRequested())
        {
            drive.updatePoseEstimate();

            Pose2D pos = drive.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.addData("Elapsed time: ", getRuntime());

            telemetry.update();
        }
    }
}
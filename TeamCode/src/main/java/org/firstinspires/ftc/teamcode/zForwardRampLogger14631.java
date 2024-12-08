package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(group = "A")
public class zForwardRampLogger14631 extends LinearOpMode
{

    double powerPerSec = 0.1;
    double maxPower = 0.9;

    private double power (double seconds)
    {
        return(Math.min(maxPower, powerPerSec * seconds));
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0), this);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this);

        waitForStart();

        MidpointTimer timer = new MidpointTimer();


        while (!isStopRequested())
        {
            drive.updatePoseEstimate();

            double power = power(timer.seconds());

            drive.leftBack.setPower(power);
            drive.leftFront.setPower(power);
            drive.rightBack.setPower(power);
            drive.rightFront.setPower(power);

            telemetry.addData("X Encoder", drive.odo.getEncoderX());

            Pose2D pos = drive.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.addData("Elapsed time: ", getRuntime());

            telemetry.update();
        }
    }
    class MidpointTimer {
        private final long beginTs = System.nanoTime();
        private long lastTime = 0;

        public double seconds() {
            return 1e-9 * (System.nanoTime() - beginTs);
        }

        public double addSplit() {
            long time = System.nanoTime() - beginTs;
            double midTimeSecs = 0.5e-9 * (lastTime + time);
            lastTime = time;
            return midTimeSecs;
        }
    }
}
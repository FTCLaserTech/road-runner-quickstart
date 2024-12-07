package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.Profiles.constantProfile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.TimeProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(group = "A")
public class zManualFeedforwardTuner14631 extends LinearOpMode
{
    double DISTANCE = 64.0;

    enum Mode
    {
        DRIVER_MODE,
        TUNING_MODE
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        double stickForward;
        double stickSideways;
        double stickForwardRotated;
        double stickSidewaysRotated;
        double adjustedAngle;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0), this);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this, ExtraOpModeFunctions.FieldSide.BLUE);

        //MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TimeProfile profile = new TimeProfile(constantProfile(DISTANCE, 0.0, MecanumDrive.PARAMS.maxWheelVel, MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel).baseProfile);

        Mode mode = Mode.TUNING_MODE;

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        boolean movingForwards = true;
        double startTs = System.nanoTime() / 1e9;
        MotorFeedforward mff = new MotorFeedforward(MecanumDrive.PARAMS.kS, MecanumDrive.PARAMS.kV, MecanumDrive.PARAMS.kA);

        while (!isStopRequested())
        {
            telemetry.addData("mode", mode);
            switch(mode)
            {
                case TUNING_MODE:
                    if (gamepad1.y)
                    {
                        mode = Mode.DRIVER_MODE;
                    }

                    double odoVelX = drive.odo.getVelX();
                    telemetry.addData("vx", MecanumDrive.PARAMS.inPerTick * odoVelX);

                    double ts = System.nanoTime() / 1e9;
                    double t = ts - startTs;
                    if (t > profile.duration)
                    {
                        movingForwards = !movingForwards;
                        startTs = ts;
                    }

                    DualNum<Time> v = profile.get(t).drop(1);
                    if (!movingForwards)
                    {
                        v = v.unaryMinus();
                    }
                    telemetry.addData("vref", v.get(0));

                    double power = mff.compute(v) / drive.voltageSensor.getVoltage();
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0.0), 0.0));
                    break;

                case DRIVER_MODE:
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        startTs = System.nanoTime() / 1e9;
                    }

                    drive.odo.update();
                    adjustedAngle = drive.odo.getHeading() + (Math.PI / 2);

                    stickSideways = gamepad1.left_stick_x;
                    stickForward = -gamepad1.left_stick_y;
                    stickSidewaysRotated = (stickSideways * Math.cos(-adjustedAngle)) - (stickForward * Math.sin(-adjustedAngle));
                    stickForwardRotated = (stickSideways * Math.sin(-adjustedAngle)) + (stickForward * Math.cos(-adjustedAngle));

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    stickSidewaysRotated,
                                    stickForwardRotated
                            ),
                            -gamepad1.right_stick_x
                    ));

                    break;
            }  // end switch

            drive.updatePoseEstimate();

            Pose2D pos = drive.odo.getPosition();
            String odoData = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", odoData);

            telemetry.addData("Elapsed time: ", getRuntime());

            telemetry.update();
        }
    }
}
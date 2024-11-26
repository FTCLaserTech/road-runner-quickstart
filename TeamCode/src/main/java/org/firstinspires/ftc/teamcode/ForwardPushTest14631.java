package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;

@TeleOp(group = "A")
public class ForwardPushTest14631 extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0), this);
        ExtraOpModeFunctions extras = new ExtraOpModeFunctions(hardwareMap, this, ExtraOpModeFunctions.FieldSide.BLUE);
        //TrajectoryBook book = new TrajectoryBook(drive, extras);

        int IMUReset = 0;
        int elevatorBottomThreshold = 200;
        double stickForward;
        double stickSideways;
        double stickForwardRotated;
        double stickSidewaysRotated;
        double adjustedAngle;
        double speedMultiplier;

        boolean gp2_left_trigger_pressed = false;
        boolean gp2_left_bumper_pressed = false;
        boolean gp2_dpad_left_pressed = false;
        boolean gp2_dpad_right_pressed = false;
        boolean gp2_dpad_up_pressed = false;
        boolean gp2_dpad_down_pressed = false;
        boolean gp2_right_bumper_pressed = false;
        boolean gp2_right_trigger_pressed = false;
        boolean gp2_right_stick_y_neg_pressed = false;
        boolean gp2_right_stick_y_pos_pressed = false;
        boolean gp2_a_pressed = false;
        boolean gp2_b_pressed = false;
        boolean gp2_y_pressed = false;
        boolean gp1_y_pressed = false;
        boolean gp1_a_pressed = false;
        boolean gp2_x_pressed = false;
        boolean gp2_by_pressed = false;


        boolean elevatorStopped = true;
        boolean elevatorBottom = true;
        boolean intakeOn = false;
        boolean manualStickMove = false;
        boolean intakeElevatorShouldMove = false;
        boolean elevatorResetOK= false;

        double elevMultMin = 0.5;
        double elevMult = 0;
        double elevHeightMax = 800;
        double slope;
        double elevatorEncoderCounts;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double stickRotation;

        double currentAmps;
        double currentAmps1;
        double currentAmpsRight;
        double maxAmps = 0;
        int numDangerAmps = 0;

        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //extras.wristMiddle();
        //extras.clawOpen();

        waitForStart();

        while (!isStopRequested())
        {


            currentAmps = extras.elevator.getCurrent(CurrentUnit.AMPS);

            if (currentAmps > maxAmps)
            {
                maxAmps = currentAmps;
            }

            if (currentAmps >= 7)
            {
                numDangerAmps += 1;

                telemetry.addLine("WARNING: HIGH AMPS");

                if(numDangerAmps >= 10)
                {
                    // in future, ONLY stop arm motion
                    stop();
                }
            }
            else
            {
                numDangerAmps = 0;
            }

            currentAmps1 = extras.arm.getCurrent(CurrentUnit.AMPS);

            if (currentAmps1 > maxAmps)
            {
                maxAmps = currentAmps;
            }

            if (currentAmps1 >= 7)
            {
                numDangerAmps += 1;

                telemetry.addLine("WARNING: HIGH AMPS");

                if(numDangerAmps >= 10)
                {
                    // in future, ONLY stop arm motion
                    stop();
                }
            }
            else
            {
                numDangerAmps = 0;
            }



            if (gamepad1.left_bumper)
                speedMultiplier = 0.5;
            else
                speedMultiplier = 1.0;

            //adjustedAngle = 0;
            //adjustedAngle = extras.adjustAngleForDriverPosition(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), ExtraOpModeFunctions.RobotStartPosition.STRAIGHT);
            drive.odo.update();
            adjustedAngle = drive.odo.getHeading() + (Math.PI / 2);


            stickSideways = gamepad1.left_stick_x * speedMultiplier;
            stickForward = -gamepad1.left_stick_y * speedMultiplier;
            stickSidewaysRotated = (stickSideways * Math.cos(-adjustedAngle)) - (stickForward * Math.sin(-adjustedAngle));
            stickForwardRotated = (stickSideways * Math.sin(-adjustedAngle)) + (stickForward * Math.cos(-adjustedAngle));

            switch (0) {
                case 0:
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    stickSidewaysRotated,
                                    stickForwardRotated
                            ),
                            -gamepad1.right_stick_x
                    ));
                    break;
                case 1:
                    // Reading joystick imputs
                    stickRotation = gamepad1.right_stick_x;

                    frontLeftPower = stickForwardRotated - stickSidewaysRotated + stickRotation;
                    frontRightPower = stickForwardRotated + stickSidewaysRotated - stickRotation;
                    backLeftPower = stickForwardRotated + stickSidewaysRotated + stickRotation;
                    backRightPower = stickForwardRotated - stickSidewaysRotated - stickRotation;

                    ArrayList<Double> motorList = new ArrayList<>(Arrays.asList(frontLeftPower, frontRightPower, backLeftPower, backRightPower));

                    double highest = 0;

                    // Find the highest value out of the 4 motor values
                    for (double n : motorList)
                    {
                        if (Math.abs(n) > highest) {
                            highest = Math.abs(n);
                        }
                    }

                    if (highest < 1)
                    {
                        highest = 1;
                    }


                    // Normalize each of the values
                    frontLeftPower = (frontLeftPower / highest) * (speedMultiplier / 10);
                    frontRightPower = (frontRightPower / highest) * (speedMultiplier / 10);
                    backLeftPower = (backLeftPower / highest) * (speedMultiplier / 10);
                    backRightPower = (backRightPower / highest) * (speedMultiplier / 10);

                    drive.leftFront.setPower(frontLeftPower);
                    drive.rightFront.setPower(frontRightPower);
                    drive.leftBack.setPower(backLeftPower);
                    drive.rightBack.setPower(backRightPower);
                    break;
            }


            drive.updatePoseEstimate();

            telemetry.addData("step", extras.step);
            telemetry.addData("ODO angle", adjustedAngle);

            Pose2D pos = drive.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            telemetry.addData("Position", data);
            telemetry.addData("Elapsed time: ", getRuntime());

            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//imports from the Mecanum website
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;

@TeleOp(group = "a")
public class BasicTeleOp extends LinearOpMode
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

            if (gamepad1.right_bumper)
                speedMultiplier = 0.6;
            else if (gamepad1.left_bumper)
                speedMultiplier = 0.4;
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


            slope = -elevMultMin / elevHeightMax;
            elevatorEncoderCounts = (extras.elevator.getCurrentPosition());
            elevMult = slope * elevatorEncoderCounts + 1;

            if (gamepad1.right_bumper)
            {
                speedMultiplier = 0.6 * elevMult;
            }
            else if (gamepad1.left_bumper)
            {
                speedMultiplier = 0.4 * elevMult;
            }
            else
            {
                speedMultiplier = 0.75 * elevMult;
            }

            // MANUAL ELEVATOR CONTROL- gamepad 2
            // stop if the limit switch is pressed

            float elevatorStick = gamepad2.left_stick_y;
            if(extras.elevatorLimit.isPressed())
            {
                extras.elevator.setPower(0);
                elevatorStopped = true;

                // it's OK to move up if the limit switch is pressed
                if(elevatorStick < 0)
                {
                    extras.elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevator.setPower(-elevatorStick);
                    elevatorStopped = false;
                }
                else if (gamepad2.right_stick_y < 0)
                {
                    extras.elevatorHigh();
                }

            }
            // don't go above the max height
            else if((extras.elevator.getCurrentPosition() > elevHeightMax) && (elevatorStick < 0))
            {
                extras.elevator.setPower(0);
                elevatorStopped = true;

                int elevPos1 = extras.elevator.getCurrentPosition();
                extras.elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extras.elevator.setTargetPosition(elevPos1);
                extras.elevator.setPower(1.0);
            }

            else
            {
                // If stick is not moved, only set power to 0 once
                if((elevatorStick == 0) && !elevatorStopped)
                {
                    extras.elevator.setPower(0);
                    elevatorStopped = true;

                    int elevPos1 = extras.elevator.getCurrentPosition();
                    extras.elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extras.elevator.setTargetPosition(elevPos1);
                    extras.elevator.setPower(1.0);

                }
                else if (elevatorStick != 0)
                {
                    extras.elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extras.elevator.setPower(-elevatorStick);
                    elevatorStopped = false;
                }
            }

            // Elevator to top with right stick up
            if (gamepad2.right_stick_y < 0)
            {
                extras.elevatorHigh();
            }

            // Elevator to bottom with right sitck down
            if ((gamepad2.right_stick_y > 0))
            {
                extras.elevatorGround();
            }

            drive.updatePoseEstimate();

            if (gamepad2.dpad_down)
            {
                extras.otherElevatorTest();
            }
            else if (gamepad2.dpad_up)
            {
                extras.elevatorTest();
            }
            else
            {
                extras.elevatorOff();
            }

            // Manual elevator movements with the left analog stick on the second gamepad (gamepad2) - added by JB
            //float elevatorStick = gamepad2.left_stick_y;
            /*
            if (extras.elevatorLimit.isPressed() && (elevatorStick > 0))
            {
                //stops the elevator if the limit switch is pressed
                if(false)
                {
                    extras.elevatorLeft.setPower(0);
                    extras.elevatorRight.setPower(0);
                    elevatorStopped = true;
                    manualStickMove = false;
                    extras.elevatorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    extras.elevatorLeft.setTargetPosition(0);
                    extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    extras.elevatorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    extras.elevatorRight.setTargetPosition(0);
                    extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                }
            }
            else if (elevatorStick != 0)
            {
                //moves the elevator with the left joystick
                extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                extras.elevatorLeft.setPower(-elevatorStick*0.2);
                extras.elevatorRight.setPower(-elevatorStick*0.2);
                elevatorStopped = false;
                manualStickMove = true;
            }
            //This condition should only be called once. When we loop back through the IF statement, no condition should be true on the second round (should we do nothing)
            else if (manualStickMove && (elevatorStick == 0))
            {
                //Only executes when we stop moving the stick
                extras.elevatorLeft.setTargetPosition(extras.elevatorLeft.getCurrentPosition());
                extras.elevatorRight.setTargetPosition(extras.elevatorRight.getCurrentPosition());
                extras.elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extras.elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                extras.elevatorLeft.setPower(100);
                extras.elevatorRight.setPower(100);
                manualStickMove = false;
                elevatorStopped = true;
            }
             */

            // reset the elevator if the limit switch is pressed
            if(extras.elevatorLimit.isPressed() && elevatorResetOK)
            {
                elevatorResetOK = false;

                // only do it if both encoders are not close to 0
                if( (Math.abs(extras.elevator.getCurrentPosition()) > 4) )
                {
                    extras.elevator.setPower(0.0);
                    //extras.armState = ExtraOpModeFunctions.ArmState.POSITIONIN;
                    extras.elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    extras.elevator.setTargetPosition(0);
                }
            }
            else if (!extras.elevatorLimit.isPressed())
            {
                elevatorResetOK = true;
            }


            if ((elevatorStick > 0) && !extras.elevatorLimit.isPressed())
            {
                extras.elevator.setTargetPosition(extras.elevator.getTargetPosition() - 10);
                //extras.elevatorLeft.setPower(1.0);
                //extras.elevatorRight.setPower(1.0);
            }
            else if (elevatorStick < 0)
            {
                extras.elevator.setTargetPosition(extras.elevator.getTargetPosition() + 10);
                //extras.elevatorLeft.setPower(1.0);
                //extras.elevatorRight.setPower(1.0);
                //elevatorStopped = false;
            }

            /*
            // RESET IMU
            if ((gamepad1.back) && (gamepad1.b))
            {
                IMUReset = 1;
            }
            else if (IMUReset == 1)
            {
                IMUReset = 0;
                telemetry.addLine("IMU Resetting...");
                telemetry.update();

                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        drive.PARAMS.logoFacingDirection,
                        drive.PARAMS.usbFacingDirection));
                //drive.imu.initialize(parameters);

                sleep(1000);
                //drive.imu.resetYaw();
            }

            // Init Elevator
            if ((gamepad2.back) && (gamepad2.y))
            {
                gp2_by_pressed = true;
            }
            else if (!gamepad2.y && gp2_by_pressed)
            {
                //extras.initElevator();
                gp2_by_pressed = false;
                extras.elevator.setPower(0.0);
                extras.elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                extras.elevator.setTargetPosition(0);

                extras.elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            }
            */
            //extend arm
            if (gamepad2.y)
            {
                extras.armExtend();
            }
            //retract arm
            else if (gamepad2.a && !gamepad2.start)
            {
                extras.armRetract();
            }

            if (gamepad2.right_bumper)
            {
                extras.intakeReverse();
                intakeOn = true;
            }
            else if (gamepad2.right_trigger > 0)
            {
                extras.intakeForward();
                intakeOn = true;
            }
            else
            {
                extras.intakeOff();
            }

            if (gamepad2.left_bumper)
            {
                gp2_left_bumper_pressed = true;
            }
            else if (!gamepad2.left_bumper && gp2_left_bumper_pressed) {
                gp2_left_bumper_pressed = false;
                extras.tailUp();
            }

            if (gamepad2.left_trigger > 0)
            {
                gp2_left_trigger_pressed = true;
            }
            else if ((gamepad2.left_trigger == 0) && gp2_left_trigger_pressed) {
                gp2_left_trigger_pressed = false;
                extras.tailDown();
            }

            if(gamepad2.dpad_right)
            {
            gp2_dpad_right_pressed = true;
            }
            else if (!gamepad2.dpad_right && gp2_dpad_right_pressed) {
                gp2_dpad_right_pressed = false;
                extras.sampleDump();
            }

            if (gamepad2.dpad_left)
            {
                gp2_dpad_left_pressed = true;
            }
            else if ((gamepad2.dpad_left) && gp2_dpad_left_pressed) {
                gp2_dpad_left_pressed = false;
                extras.samplePickup();

            }


                //extras.setLeds(getRuntime());

            telemetry.addData("step", extras.step);
            //telemetry.addData("x", drive.pose.position.x);
            //telemetry.addData("y", drive.pose.position.y);
            //telemetry.addData("heading", drive.pose.heading.real);
            telemetry.addData("IMU angle", adjustedAngle);

            Pose2D pos = drive.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            telemetry.addData("Position", data);
            telemetry.addLine();
            telemetry.addData("Arm Counts", extras.arm.getCurrentPosition());
            telemetry.addData("Elevator encoder counts: ", extras.elevator.getCurrentPosition());
            telemetry.addData("Elevator current: ", currentAmps);
            telemetry.addData("Max amps: ", maxAmps);
            telemetry.addData("Elevator limit: ", extras.elevatorLimit.isPressed());
            //telemetry.addData("Elevator stopped? ", elevatorStopped);
            //telemetry.addData("lift position", extras.lift.getCurrentPosition());

            //telemetry.addData("LF RF LB RB: ", "%.1f %.1f %.1f %.1f", drive.leftFront.getPower(), drive.rightFront.getPower(), drive.leftBack.getPower(), drive.rightBack.getPower());
            telemetry.addData("Elapsed time: ", getRuntime());

            telemetry.update();
        }
    }
}
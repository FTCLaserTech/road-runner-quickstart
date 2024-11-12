package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Config


public class ExtraOpModeFunctions
{
    public enum Signal {LEFT, MIDDLE, RIGHT}
    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};
    public enum FieldSide {RED, BLUE}

    public FieldSide fs = FieldSide.BLUE;

    int numRed = 0;
    int numGreen = 0;
    int numBlue = 0;

    final boolean USING_WEBCAM = true;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;


    public enum ElevatorPosition {COLLECT, GROUND, LOW, MIDDLE, HIGH, TWO, THREE, FOUR, FIVE}
    public ElevatorPosition elevatorPosition = ElevatorPosition.COLLECT;
    public enum ElbowPosition {EXTEND, RETRACT, PURPLE}
    public ElbowPosition elbowPosition = ElbowPosition.RETRACT;
    public static final double PI = 3.14159265;
    public int elevatorTargetPosition = 0;

    public int elevatorHoverPos = 100;
    public int target = 0;


    //private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;

    public Servo elbow;
    public LinearOpMode localLop = null;

    public DcMotorEx elevator;
    public CRServo intake;
    public DcMotorEx arm;
    public TouchSensor elevatorLimit;
    public Servo tail;
    public Servo dumper;


    public RevColorSensorV3 colorSensor;
    public ColorRangeSensor testColorSensor;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    public HardwareMap hm = null;
    public LinearOpMode lop = null;

    public ExtraOpModeFunctions(HardwareMap hardwareMap, LinearOpMode linearOpMode, FieldSide fieldSide)
    {
        hm = hardwareMap;
        lop = linearOpMode;
        fs = fieldSide;

        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(CRServo.Direction.FORWARD);
        intake.setPower(0);

        tail = hardwareMap.get(Servo.class, "tail");

        dumper = hardwareMap.get(Servo.class, "dumper");

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        elevator = hardwareMap.get(DcMotorEx.class, "elevator");


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10.0, 0.05, 0.0, 0.0);
        elevator.setDirection(DcMotorEx.Direction.FORWARD);
        elevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
        blinkinLedDriver.setPattern(pattern);
    }

    public void initElevator()
    {
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setPower(0.2);
        localLop.sleep(600);
        elevator.setPower(0);


        localLop.telemetry.addData("Limit ", elevatorLimit.getValue());
        //localLop.telemetry.update();

        localLop.sleep(100);

        elevator.setPower(-0.1);


        while(!elevatorLimit.isPressed())
        {
            ;
        }

        elevator.setPower(0);

        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        localLop.sleep(250);

        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        /*
        elevatorLeft.setTargetPosition(20);
        elevatorRight.setTargetPosition(20);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);

         */

        localLop.telemetry.addLine("Elevator Initialized!");
        localLop.telemetry.update();

    }


    public void elbowMove (int distance)
    {
        elbow.setPosition(elbow.getPosition() + distance);
    }

    public void intakeForward() {
        intake.setPower(1);
    }
    public void intakeReverse() {
        intake.setPower(-1);
    }
    public void intakeOff()
    {
        intake.setPower(0);
    }

    public void armExtend()
    {
        target = 1500;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target);
        arm.setPower(1.0);
    }
    public void armRetract()
    {
        target = 0;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target);
        arm.setPower(1.0);
    }

    public void otherElevatorTest()
    {
        arm.setPower(0.1);
    }
    public void elevatorTest()
    {
        arm.setPower(-0.1);
    }
    public void elevatorOff()
    {
        arm.setPower(0);
    }

    public void armOff() {
        arm.setPower(0);
    }

    /*
    public void setElevatorPosition(int target)
    {
        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);
        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
    }
    */



    public void elevatorGround()
    {
        target = 0;
        elevatorPosition = elevatorPosition.COLLECT;

        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(target);

        elevator.setPower(1.0);
    }

    public void elevatorJunction()
    {
        target = 20;
        elevatorPosition = elevatorPosition.GROUND;

        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(target);

        elevator.setPower(1.0);
    }

    public void elevatorLow()
    {
        target = 1210;
        elevatorPosition = elevatorPosition.LOW;

        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(target);

        elevator.setPower(1.0);
    }

    public void elevatorMiddle()
    {
        target = 2020;
        elevatorPosition = elevatorPosition.MIDDLE;

        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(target);

        elevator.setPower(1.0);
    }

    public void elevatorHigh()
    {
        target = 2820;
        elevatorPosition = elevatorPosition.HIGH;

        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(target);

        elevator.setPower(1.0);
    }

    public void tailUp()
    {
        tail.setPosition(5);
    }

    public void tailDown()
    {
        tail.setPosition(1);
    }

    public boolean armMoving = false;
    public int step = 0;

    public double adjustAngleForDriverPosition(double angle, RobotStartPosition robotStartPosition)
    {
        switch (robotStartPosition)
        {
            case STRAIGHT:
                angle = angle + PI/2;
                if(angle > (PI))
                    angle = angle - (PI*2);
                break;
            case LEFT:
                angle = angle - PI/2;
                if(angle < (-PI))
                    angle = angle + (PI*2);
                break;
            case RIGHT:
                angle = angle + PI/2;
                if(angle > (PI))
                    angle = angle - (PI*2);
                break;
        }
        return angle;
    }
}
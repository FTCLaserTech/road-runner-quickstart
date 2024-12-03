package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


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

    public boolean firstPressed = true;



    public RevColorSensorV3 colorSensor;
    public ColorRangeSensor testColorSensor;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    public HardwareMap hm = null;
    public LinearOpMode lop = null;

    public enum BasketDelivery {IDLE, ARMUP, WAIT1, INTAKEOUT, WAIT2, INTAKEOFF}
    BasketDelivery basketDeliveryState = BasketDelivery.IDLE;

    public enum Collect {IDLE, DUMP, WAIT, COLLECT}
    Collect dumpState = Collect.IDLE;


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
        elevator.setDirection(DcMotorEx.Direction.REVERSE);
        elevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
        blinkinLedDriver.setPattern(pattern);
    }

    ElapsedTime basketDeliveryStateTimer = new ElapsedTime(MILLISECONDS);
    public void basketDeliveryStateMachine() {
        switch (basketDeliveryState) {
            case IDLE:
                //do nothing
                break;
            case ARMUP:
                armRetract();
                intakeIn();
                tailDown();
                basketDeliveryStateTimer.reset();
                basketDeliveryState = BasketDelivery.WAIT1;
                break;
            case WAIT1:
                if (basketDeliveryStateTimer.milliseconds() > 700) {
                    basketDeliveryState = BasketDelivery.INTAKEOUT;
                }
                break;
            case INTAKEOUT:
                intakeOut();
                basketDeliveryStateTimer.reset();
                basketDeliveryState = BasketDelivery.WAIT2;
                break;
            case WAIT2:
                if (basketDeliveryStateTimer.milliseconds() > 1000) {
                    basketDeliveryState = BasketDelivery.INTAKEOFF;
                }
                break;
            case INTAKEOFF:
                intakeOff();
                elevatorHighBasket();
                basketDeliveryState = BasketDelivery.IDLE;
                break;

        }
    }

    ElapsedTime dumpStateTimer = new ElapsedTime(MILLISECONDS);
    public void collectStateMachine() {
        switch (dumpState) {
            case IDLE:
                //do nothing
                break;
            case DUMP:
                dumper.setPosition(1.0);
                dumpStateTimer.reset();
                dumpState = Collect.WAIT;
                break;
            case WAIT:
                if (dumpStateTimer.milliseconds() > 1200) {
                    dumpState = Collect.COLLECT;
                }
                break;
            case COLLECT:
                dumper.setPosition(-1.0);
                dumpState = Collect.IDLE;
                break;

        }
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


        while(elevatorLimit.isPressed())
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
    public void initArm()
    {
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setPower(-0.1);
        arm.setCurrentAlert(5, CurrentUnit.AMPS);

        while(!arm.isOverCurrent())
        {
            localLop.telemetry.addData("Current", arm.getCurrent(CurrentUnit.AMPS));
            localLop.telemetry.update();
        }

        arm.setPower(0);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        localLop.sleep(250);

        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        /*
        elevatorLeft.setTargetPosition(20);
        elevatorRight.setTargetPosition(20);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);

         */

        localLop.telemetry.addLine("Arm Initialized!");
        localLop.telemetry.update();

    }



    public void elbowMove (int distance)
    {
        elbow.setPosition(elbow.getPosition() + distance);
    }

    public void intakeIn() {
        intake.setPower(-1);
    }
    public void intakeOut() {
        intake.setPower(1);
    }
    public void intakeOff()
    {
        intake.setPower(0);
    }

    public enum ArmPosition {STOP, EXTEND, RETRACT, HORIZONTAL, VERTICAL,HANG}
    ArmPosition armPosition = ArmPosition.STOP;

    public void armExtend()
    {
        armPosition = ArmPosition.EXTEND;
        target = 1600;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target);
        arm.setPower(1.0);
    }
    public void armRetract()
    {
        armPosition = ArmPosition.RETRACT;
        target = 100;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target);
        arm.setPower(1.0);
    }
    public void armHorizontal()
    {
        armPosition = ArmPosition.HORIZONTAL;
        target = 1400;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target);
        arm.setPower(1.0);
    }
    public void armVertical()
    {
        armPosition = ArmPosition.VERTICAL;
        target = 500;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target);
        arm.setPower(1.0);
    }
    public void armHang()
    {
        armPosition = ArmPosition.HANG;
        target = 0;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(target);
        arm.setPower(1.0);
    }


    /*
    public void armExtend()
    {
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setPower(0.2);
    }
    public void armRetract()
    {
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setPower(-0.2);
    }
    public void armStop()
    {
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setPower(0.0);
    }
    */
    public void elevatorDown()
    {
        target = 100;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(target);
        elevator.setPower(1.0);
        firstPressed = true;
    }
    public void elevatorSpecimanGrab()
    {
        target = 40;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(target);
        elevator.setPower(1.0);
        firstPressed = true;
    }
    public void elevatorHighBasket()
    {
        target = 2600;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(target);
        elevator.setPower(1.0);
        firstPressed = true;
    }
    public void elevatorHighChamber()
    {
        target = 1500;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(target);
        elevator.setPower(1.0);
        firstPressed = true;
    }

    public void elevatorOff()
    {
        elevator.setPower(0);
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
        tail.setPosition(-1);
    }

    public void tailDown()
    {
        tail.setPosition(1);
    }

    public void sampleDump()
        {
        dumper.setPosition(1.0);
    }

    public void samplePickup()
    {
        dumper.setPosition(-1.0);
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
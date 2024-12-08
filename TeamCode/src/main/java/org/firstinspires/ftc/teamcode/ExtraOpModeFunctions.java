package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;


@Config


public class ExtraOpModeFunctions
{
    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};

    public enum ElevatorPosition {COLLECT, GROUND, LOW, MIDDLE, HIGH, TWO, THREE, FOUR, FIVE}
    public static final double PI = 3.14159265;

    public int elevatorTarget = 0;

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
    
    public enum BasketDelivery {IDLE, ARMUP, WAIT1, INTAKEOUT, WAIT2, INTAKEOFF}
    BasketDelivery basketDeliveryState = BasketDelivery.IDLE;

    public enum Collect {IDLE, DUMP, WAIT, COLLECT}
    Collect dumpState = Collect.IDLE;

    public enum SpecimenPickupStates {IDLE, PICKUP, WAIT, DROPTAIL}
    SpecimenPickupStates specimenPickupState = SpecimenPickupStates.IDLE;

    public ExtraOpModeFunctions(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        hm = hardwareMap;
        localLop = linearOpMode;

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

        //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10.0, 0.05, 0.0, 0.0);
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");
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

    ElapsedTime specimenPickupStateTimer = new ElapsedTime(MILLISECONDS);
    public void specimenPickupStateMachine() {
        switch (specimenPickupState) {
            case IDLE:
                //do nothing
                break;
            case PICKUP:
                armVertical();
                elevatorHighChamber();
                specimenPickupStateTimer.reset();
                specimenPickupState = SpecimenPickupStates.WAIT;
                break;
            case WAIT:
                if (dumpStateTimer.milliseconds() > 1000)
                {
                    specimenPickupState = SpecimenPickupStates.DROPTAIL;
                }
                break;
            case DROPTAIL:
                tailDown();
                specimenPickupState = SpecimenPickupStates.IDLE;
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
        elevatorTarget = 1600;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }
    public void armRetract()
    {
        armPosition = ArmPosition.RETRACT;
        elevatorTarget = 100;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }
    public void armHorizontal()
    {
        armPosition = ArmPosition.HORIZONTAL;
        elevatorTarget = 1400;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }
    public void armVertical()
    {
        armPosition = ArmPosition.VERTICAL;
        elevatorTarget = 500;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }
    public void armHang()
    {
        armPosition = ArmPosition.HANG;
        elevatorTarget = 0;
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(elevatorTarget);
        arm.setPower(1.0);
    }

    public void elevatorDown()
    {
        elevatorTarget = 100;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(elevatorTarget);
        elevator.setPower(1.0);
        firstPressed = true;
    }
    public void elevatorSpecimanGrab()
    {
        elevatorTarget = 20;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(elevatorTarget);
        elevator.setPower(1.0);
        firstPressed = true;
    }
    public void elevatorHighBasket()
    {
        elevatorTarget = 2600;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(elevatorTarget);
        elevator.setPower(1.0);
        firstPressed = true;
    }
    public void elevatorHighChamber()
    {
        elevatorTarget = 1400;
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setTargetPosition(elevatorTarget);
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

    private static final String BASE_FOLDER_NAME = "Team14631";
    private static final String TEAM_LOG = "Team14631";
    public void saveAutoStartRotation(Double angle)
    {
        Writer fileWriter;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        directory.mkdir();
        try
        {
            fileWriter = new FileWriter(directoryPath+"/"+TEAM_LOG+".csv");
            fileWriter.write(angle.toString());
            fileWriter.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
    public double readAutoStartRotation()
    {
        double angle = 0;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        directory.mkdir();
        try
        {
            BufferedReader br = new BufferedReader(new FileReader(directoryPath+"/"+TEAM_LOG+".csv"));
            angle = Double.parseDouble(br.readLine());
            br.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        return(angle);
    }
}


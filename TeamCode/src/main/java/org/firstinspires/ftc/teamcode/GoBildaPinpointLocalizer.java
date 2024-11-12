package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class GoBildaPinpointLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = 0.0; // y position of the parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    // public final Encoder par, perp;
    // public final IMU imu;
    public final GoBildaPinpointDriver odo;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    // private final double inPerTick;
    // 32mm diameter; 2000 ticks per rev
    private final static double mmPerTick = 32.0 * Math.PI / 2000.0;
    private final static double inPerTick = mmPerTick / 25.4;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public GoBildaPinpointLocalizer(HardwareMap hardwareMap, GoBildaPinpointDriver odo) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        // par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par")));
        // perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

        // TODO: reverse encoder directions if needed
        //   par.setDirection(DcMotorSimple.Direction.REVERSE);

        // this.imu = imu;
        this.odo = odo;

        // this.inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        // PositionVelocityPair parPosVel = par.getPositionAndVelocity(); // units are ticks, ticks/sec, ticks, ticks/sec
        // PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();
        PositionVelocityPair parPosVel = new PositionVelocityPair(odo.getEncoderX(), (int)(odo.getVelX()/ mmPerTick), odo.getEncoderX(), (int)(odo.getVelX()/ mmPerTick));
        PositionVelocityPair perpPosVel =  new PositionVelocityPair(odo.getEncoderY(), (int)(odo.getVelY()/ mmPerTick), odo.getEncoderY(), (int)(odo.getVelY()/ mmPerTick));

        /*
        public class YawPitchRollAngles {
            private final AngleUnit angleUnit;
            private final double yaw;
            private final double pitch;
            private final double roll;
            private final long acquisitionTime;
        */

        /*
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
        AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        AngularVelocity angularVelocity = new AngularVelocity(
                UnnormalizedAngleUnit.RADIANS,
                (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
                (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
                angularVelocityDegrees.acquisitionTime
        );
        */

        //FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        // Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
        Rotation2d heading = Rotation2d.exp(odo.getHeading());

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        // double rawHeadingVel = angularVelocity.zRotationRate;
        double rawHeadingVel = odo.getHeadingVelocity();

        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        return twist;
    }
}

package org.firstinspires.ftc.teamcode.localization.localizers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.localization.Encoder;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

public class DriveEncoderIMULocalizer extends Localizer {

    private static final double IMU_STALE_THRESHOLD = 0.00005;

    private final Encoder frontLeft;
    private final Encoder frontRight;
    private final Encoder backLeft;
    private final Encoder backRight;
    private final IMU imu;

    private final double chassisWidth;
    private final double chassisLength;

    private double headingOffset;
    private double previousRawImuHeading;

    private double prevTime, currTime;

    public DriveEncoderIMULocalizer(HardwareMap hardwareMap, DriveEncoderIMUAttributes attributes) {

        frontLeft = new Encoder(
                hardwareMap.get(DcMotorEx.class, attributes.getFrontLeftName()),
                attributes.getOdometryData()
        );
        frontRight = new Encoder(
                hardwareMap.get(DcMotorEx.class, attributes.getFrontRightName()),
                attributes.getOdometryData()
        );
        backLeft = new Encoder(
                hardwareMap.get(DcMotorEx.class, attributes.getBackLeftName()),
                attributes.getOdometryData()
        );
        backRight = new Encoder(
                hardwareMap.get(DcMotorEx.class, attributes.getBackRightName()),
                attributes.getOdometryData()
        );

        frontLeft.setDirection(attributes.getFrontLeftDirection());
        frontRight.setDirection(attributes.getFrontRightDirection());
        backLeft.setDirection(attributes.getBackLeftDirection());
        backRight.setDirection(attributes.getBackRightDirection());

        chassisWidth = attributes.getChassisWidth();
        chassisLength = attributes.getChassisLength();

        imu = hardwareMap.get(IMU.class, attributes.getImuName());
        imu.initialize(attributes.getImuParameters());
        imu.resetYaw();

        previousRawImuHeading = MathHelper.normalizeAngleRad(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        headingOffset = 0d;

        currTime = System.nanoTime() * 1e-9;
    }

    @Override
    public void setPose(Pose pose) {

        double rawHeading = MathHelper.normalizeAngleRad(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        headingOffset = MathHelper.normalizeAngleRad(pose.heading - rawHeading);
        this.pose = pose;

        frontLeft.reset();
        frontRight.reset();
        backLeft.reset();
        backRight.reset();
    }

    @Override
    public void update() {

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();

        double dLF = frontLeft.getDeltaInches();
        double dRF = frontRight.getDeltaInches();
        double dLB = backLeft.getDeltaInches();
        double dRB = backRight.getDeltaInches();

        double rawImuHeading = MathHelper.normalizeAngleRad(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        boolean imuFresh = Math.abs(MathHelper.normalizeAngleRad(rawImuHeading - previousRawImuHeading)) > IMU_STALE_THRESHOLD;
        previousRawImuHeading = rawImuHeading;

        double encoderDeltaHeading = (-dLF + dRF - dLB + dRB) / (2d * (chassisWidth + chassisLength));

        double newHeading = imuFresh
                ? MathHelper.normalizeAngleRad(rawImuHeading + headingOffset)
                : MathHelper.normalizeAngleRad(pose.heading + encoderDeltaHeading);

        double deltaHeading = MathHelper.normalizeAngleRad(newHeading - pose.heading);

        double deltaForward = (dLF + dRF + dLB + dRB) / 4d;
        double deltaStrafe = (-dLF + dRF + dLB - dRB) / 4d;

        Pose robotDeltas = new Pose(deltaForward, deltaStrafe, deltaHeading);
        Pose globalDelta = MathHelper.exponentialIntegrate(robotDeltas, pose.heading);

        pose = pose.add(globalDelta);

        prevTime = currTime;
        currTime = System.nanoTime() * 1e-9;
        deltaTime = currTime - prevTime;

        if (deltaTime <= 0) return;

        velocity = globalDelta.divideBy(deltaTime);
    }

}
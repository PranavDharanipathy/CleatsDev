package org.firstinspires.ftc.teamcode.localization.localizers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.localization.Encoder;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

public class TwoDeadWheelLocalizer extends Localizer {

    private final Encoder forwardEncoder;
    private final Encoder strafeEncoder;
    private final IMU imu;

    private final double forwardPodY;
    private final double strafePodX;

    private double previousImuHeading;

    private double prevTime, currTime;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, TwoDeadWheelAttributes attributes) {

        forwardEncoder = new Encoder(
                hardwareMap.get(DcMotorEx.class, attributes.getForwardEncoderName()),
                attributes.getOdometryPodData()
        );
        strafeEncoder = new Encoder(
                hardwareMap.get(DcMotorEx.class, attributes.getStrafeEncoderName()),
                attributes.getOdometryPodData()
        );

        forwardEncoder.setDirection(attributes.getForwardDirection());
        strafeEncoder.setDirection(attributes.getStrafeDirection());

        forwardPodY = attributes.getForwardPodY();
        strafePodX = attributes.getStrafePodX();

        imu = hardwareMap.get(IMU.class, attributes.getImuName());
        imu.initialize(attributes.getImuParameters());
        imu.resetYaw();

        previousImuHeading = MathHelper.normalizeAngleRad(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        currTime = System.nanoTime() * 1e-9;
    }

    @Override
    public void setPose(Pose pose) {
        this.pose = pose;
        forwardEncoder.reset();
        strafeEncoder.reset();
    }

    @Override
    public void update() {

        forwardEncoder.update();
        strafeEncoder.update();

        double currentImuHeading = MathHelper.normalizeAngleRad(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double deltaHeading = MathHelper.normalizeAngleRad(currentImuHeading - previousImuHeading);
        previousImuHeading = currentImuHeading;

        double forwardDelta = forwardEncoder.getDeltaInches();
        double strafeDelta = strafeEncoder.getDeltaInches();

        double correctedForward = forwardDelta + forwardPodY * deltaHeading;
        double correctedStrafe = strafeDelta - strafePodX * deltaHeading;

        Pose robotDeltas = new Pose(correctedForward, correctedStrafe, deltaHeading);
        Pose globalDelta = MathHelper.exponentialIntegrate(robotDeltas, pose.heading);

        pose = pose.add(globalDelta);

        prevTime = currTime;
        currTime = System.nanoTime() * 1e-9;
        deltaTime = currTime - prevTime;

        if (deltaTime <= 0) return;

        velocity = globalDelta.divideBy(deltaTime);
    }

}
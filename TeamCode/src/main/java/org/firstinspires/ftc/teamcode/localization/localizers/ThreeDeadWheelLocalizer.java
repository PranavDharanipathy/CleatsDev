package org.firstinspires.ftc.teamcode.localization.localizers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.localization.Encoder;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.util.MathHelper;
import org.firstinspires.ftc.teamcode.util.Pose;

public class ThreeDeadWheelLocalizer extends Localizer {

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder strafeEncoder;

    private final double leftPodY;
    private final double rightPodY;
    private final double strafePodX;

    private double prevTime, currTime;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, ThreeDeadWheelAttributes attributes) {

        leftEncoder = new Encoder(
                hardwareMap.get(DcMotorEx.class, attributes.getLeftEncoderName()),
                attributes.getOdometryPodData()
        );
        rightEncoder = new Encoder(
                hardwareMap.get(DcMotorEx.class, attributes.getRightEncoderName()),
                attributes.getOdometryPodData()
        );
        strafeEncoder = new Encoder(
                hardwareMap.get(DcMotorEx.class, attributes.getStrafeEncoderName()),
                attributes.getOdometryPodData()
        );

        leftEncoder.setDirection(attributes.getLeftDirection());
        rightEncoder.setDirection(attributes.getRightDirection());
        strafeEncoder.setDirection(attributes.getStrafeDirection());

        leftPodY = attributes.getLeftPodY();
        rightPodY = attributes.getRightPodY();
        strafePodX = attributes.getStrafePodX();

        currTime = System.nanoTime() * 1e-9;
    }

    @Override
    public void setPose(Pose pose) {
        this.pose = pose;
        leftEncoder.reset();
        rightEncoder.reset();
        strafeEncoder.reset();
    }

    @Override
    public void update() {

        leftEncoder.update();
        rightEncoder.update();
        strafeEncoder.update();

        double leftDelta = leftEncoder.getDeltaInches();
        double rightDelta = rightEncoder.getDeltaInches();
        double strafeDelta = strafeEncoder.getDeltaInches();

        double podSeparation = leftPodY - rightPodY;

        double deltaHeading = (rightDelta - leftDelta) / podSeparation;

        double deltaForward = (rightDelta * leftPodY - leftDelta * rightPodY) / podSeparation;

        double deltaStrafe = strafeDelta - strafePodX * deltaHeading;

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
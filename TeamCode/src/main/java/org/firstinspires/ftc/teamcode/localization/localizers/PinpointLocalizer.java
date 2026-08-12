package org.firstinspires.ftc.teamcode.localization.localizers;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.util.Pose;

public class PinpointLocalizer extends Localizer {

    private final GoBildaPinpointDriver localizer;

    public PinpointLocalizer(HardwareMap hardwareMap, PinpointAttributes lAttributes) {

        localizer = hardwareMap.get(GoBildaPinpointDriver.class, lAttributes.getDeviceName());

        localizer.setOffsets(lAttributes.getForwardPodOffsetMM(), lAttributes.getStrafePodOffsetMM(), DistanceUnit.MM);

        double mmPerTick = lAttributes.getOdometryPodData().getInchesPerTick() * 25.4;
        localizer.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);

        localizer.setEncoderDirections(lAttributes.getForwardPodDirection(), lAttributes.getStrafePodDirection());
        localizer.resetPosAndIMU();

        localizer.setPosition(new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS, 0));

        currTime = System.nanoTime() * 1e-9;
    }

    private double prevTime, currTime;

    @Override
    public void setPose(Pose pose) {

        localizer.setPosition(Pose.poseToPose2D(pose));
        this.pose = pose;
    }

    @Override
    public void update() {

        localizer.update();
        pose = Pose.pose2DToPose(localizer.getPosition());

        prevTime = currTime;
        currTime = System.nanoTime() * 1e-9;
        deltaTime = currTime - prevTime;

        if (deltaTime <= 0) return;

        velocity = new Pose(localizer.getVelX(DistanceUnit.INCH), localizer.getVelY(DistanceUnit.INCH), localizer.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized()));

    }

}
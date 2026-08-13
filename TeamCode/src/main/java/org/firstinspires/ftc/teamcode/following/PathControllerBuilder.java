package org.firstinspires.ftc.teamcode.following;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.following.chassis.Chassis;
import org.firstinspires.ftc.teamcode.following.chassis.MotionConstraints;
import org.firstinspires.ftc.teamcode.following.config.ChassisMotorDirectionsConfig;
import org.firstinspires.ftc.teamcode.following.config.ChassisMotorNamesConfig;
import org.firstinspires.ftc.teamcode.following.config.FinalLocalizerNKFConfig;
import org.firstinspires.ftc.teamcode.localization.FinalLocalizer;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.util.Lazy;

import java.util.function.Supplier;

public class PathControllerBuilder {

    private final HardwareMap hardwareMap;
    private ChassisMotorNamesConfig chassisMotorNamesConfig;
    private ChassisMotorDirectionsConfig chassisMotorDirectionsConfig;

    private Lazy<Localizer> localizer;

    private FinalLocalizerNKFConfig velocityXConfig, velocityYConfig, velocityHeadingConfig;
    private FinalLocalizerNKFConfig accelerationXConfig, accelerationYConfig, accelerationHeadingConfig;
    private FinalLocalizerNKFConfig jerkXConfig, jerkYConfig, jerkHeadingConfig;

    private MotionConstraints motionConstraints;

    public PathControllerBuilder(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public PathControllerBuilder chassisMotorNamesConfig(ChassisMotorNamesConfig config) {
        chassisMotorNamesConfig = config;
        return this;
    }

    public PathControllerBuilder chassisMotorDirectionsConfig(ChassisMotorDirectionsConfig config) {
        chassisMotorDirectionsConfig = config;
        return this;
    }

    public PathControllerBuilder localizer(Supplier<Localizer> localizer) {
        this.localizer = new Lazy<>(localizer);
        return this;
    }

    public PathControllerBuilder velocityXNKFParams(FinalLocalizerNKFConfig config) {
        velocityXConfig = config;
        return this;
    }

    public PathControllerBuilder velocityYNKFParams(FinalLocalizerNKFConfig config) {
        velocityYConfig = config;
        return this;
    }

    public PathControllerBuilder velocityHeadingNKFParams(FinalLocalizerNKFConfig config) {
        velocityHeadingConfig = config;
        return this;
    }

    public PathControllerBuilder accelerationXNKFParams(FinalLocalizerNKFConfig config) {
        accelerationXConfig = config;
        return this;
    }

    public PathControllerBuilder accelerationYNKFParams(FinalLocalizerNKFConfig config) {
        accelerationYConfig = config;
        return this;
    }

    public PathControllerBuilder accelerationHeadingNKFParams(FinalLocalizerNKFConfig config) {
        accelerationHeadingConfig = config;
        return this;
    }

    public PathControllerBuilder jerkXNKFParams(FinalLocalizerNKFConfig config) {
        jerkXConfig = config;
        return this;
    }

    public PathControllerBuilder jerkYNKFParams(FinalLocalizerNKFConfig config) {
        jerkYConfig = config;
        return this;
    }

    public PathControllerBuilder jerkHeadingNKFParams(FinalLocalizerNKFConfig config) {
        jerkHeadingConfig = config;
        return this;
    }

    public PathControllerBuilder motionConstraints(MotionConstraints motionConstraints) {
        this.motionConstraints = motionConstraints;
        return this;
    }

    public PathController build() {

        FinalLocalizer localizer = new FinalLocalizer(this.localizer.get());

        localizer.setNoiseFilterParameters(
            velocityXConfig.assemble(), velocityYConfig.assemble(), velocityHeadingConfig.assemble(),
            accelerationXConfig.assemble(), accelerationYConfig.assemble(), accelerationHeadingConfig.assemble(),
            jerkXConfig.assemble(), jerkYConfig.assemble(), jerkHeadingConfig.assemble()
        );

        //TODO allat!!
        return new PathController(
                new Chassis(
                        hardwareMap,
                        chassisMotorNamesConfig.assemble(),
                        chassisMotorDirectionsConfig.assemble(),
                        motionConstraints.getAmaxF() / motionConstraints.getVmaxF()
                ),
                localizer,
                motionConstraints
        );
    }

}

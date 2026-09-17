package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.following.PathControllerBuilder;
import org.firstinspires.ftc.teamcode.following.PoseLQRController;
import org.firstinspires.ftc.teamcode.following.PrecisionModeThresholds;
import org.firstinspires.ftc.teamcode.following.chassis.BrakingModel;
import org.firstinspires.ftc.teamcode.following.chassis.MotionConstraints;
import org.firstinspires.ftc.teamcode.following.config.ChassisMotorDirectionsConfig;
import org.firstinspires.ftc.teamcode.following.config.ChassisMotorNamesConfig;
import org.firstinspires.ftc.teamcode.following.config.FinalLocalizerNKFConfig;
import org.firstinspires.ftc.teamcode.localization.Odometer;
import org.firstinspires.ftc.teamcode.localization.localizers.PinpointAttributes;
import org.firstinspires.ftc.teamcode.localization.localizers.PinpointLocalizer;

public class Constants {

    public static PathController getPathController(HardwareMap hardwareMap) {

        return new PathControllerBuilder(hardwareMap)
                .chassisMotorDirectionsConfig(
                        new ChassisMotorDirectionsConfig(
                                DcMotorSimple.Direction.REVERSE,
                                DcMotorSimple.Direction.FORWARD,
                                DcMotorSimple.Direction.REVERSE,
                                DcMotorSimple.Direction.FORWARD
                        )
                )
                .chassisMotorNamesConfig(
                        new ChassisMotorNamesConfig(
                                "fl",
                                "fr",
                                "bl",
                                "br"
                        )
                )
                .localizer(() -> new PinpointLocalizer(
                        hardwareMap,
                        new PinpointAttributes(
                                "pinpoint",
                                -83.57,
                                19.73,
                                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                Odometer.createGobildaFourBarPod()
                        )
                ))
                .velocityXNKFParams(new FinalLocalizerNKFConfig(1, 0, 1))
                .velocityYNKFParams(new FinalLocalizerNKFConfig(1, 0, 1))
                .velocityHeadingNKFParams(new FinalLocalizerNKFConfig(1, 0, 1))
                .accelerationXNKFParams(new FinalLocalizerNKFConfig(1, 0, 1))
                .accelerationYNKFParams(new FinalLocalizerNKFConfig(1, 0, 1))
                .accelerationHeadingNKFParams(new FinalLocalizerNKFConfig(1, 0, 1))
                .motionConstraints(
                        new MotionConstraints(
                                0, 0, 0, 0,
                                0, 0, 0, 0,
                                0, 0, 0, 0
                        )
                )
                .brakingModel(() -> new BrakingModel(
                                new double[] {10, 20, 30, 40, 50},
                                new double[] {0, 0, 0, 0, 0},
                                new double[] {0, 0, 0, 0, 0},
                                new double[] {0, 0, 0, 0, 0},
                                new double[] {2, 4, 6, 8},
                                new double[] {0, 0, 0, 0},
                                0,
                                0,
                                0
                ))
                .poseLQRController(() -> new PoseLQRController(
                        0, 0,
                        0, 0,
                        0, 0
                ))
                .precisionModeThresholds(
                        new PrecisionModeThresholds(
                                0,0,
                                0,0,
                                0,0,
                                0,0
                        )
                )
                .build();
    }
}

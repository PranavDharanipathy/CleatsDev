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
                                60.6171, 50.7923, 37.7414, 5.0722,
                                163.4158, 109.4816, 110.4941, 34.7167,
                                163.4158, 140.7712, 119.3241, 34.7167
                        )
                )
                .brakingModel(() -> new BrakingModel(
                                new double[] {6.2902, 12.5805, 18.8707, 25.1609, 31.4512},
                                new double[] {0.4266, 1.1849, 2.0496, 3.4624, 4.3324},
                                new double[] {0.3523, 1.0566, 1.8670, 3.1063, 5.5814},
                                new double[] {0.4342, 0.9660, 2.2971, 3.7640, 4.8539},
                                new double[] {0.8454, 1.6907, 2.5361, 3.3815, 4.2268},
                                new double[] {0.0386, 0.0993, 0.1418, 0.2478, 0.2874},
                                0.067167684,
                                Math.toRadians(0.858448111)
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

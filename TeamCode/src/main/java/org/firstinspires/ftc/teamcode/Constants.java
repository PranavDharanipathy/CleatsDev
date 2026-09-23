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
                                new double[] {7.5483, 15.0966, 22.6448, 30.1931, 37.7414},
                                new double[] {0.4473, 1.5288, 2.3919, 4.9343, 6.3572},
                                new double[] {0.6015, 1.4211, 2.8760, 4.3038, 5.0928},
                                new double[] {0.4197, 1.3682, 2.5735, 4.8332, 5.8095},
                                new double[] {1.0144, 2.0289, 3.0433, 4.0578, 5.0722},
                                new double[] {0.0483, 0.1186, 0.1794, 0.3137, 0.3607},
                                0.354803287,
                                Math.toRadians(4.95159065)
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

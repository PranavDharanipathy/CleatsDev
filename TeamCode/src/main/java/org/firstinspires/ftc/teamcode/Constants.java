package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.following.PathController;
import org.firstinspires.ftc.teamcode.following.PathControllerBuilder;
import org.firstinspires.ftc.teamcode.following.PoseLQRController;
import org.firstinspires.ftc.teamcode.following.PrecisionModeThresholds;
import org.firstinspires.ftc.teamcode.following.chassis.MotionConstraints;
import org.firstinspires.ftc.teamcode.following.config.ChassisMotorDirectionsConfig;
import org.firstinspires.ftc.teamcode.following.config.ChassisMotorNamesConfig;
import org.firstinspires.ftc.teamcode.following.config.FinalLocalizerNKFConfig;
import org.firstinspires.ftc.teamcode.localization.Encoder;
import org.firstinspires.ftc.teamcode.localization.Odometer;
import org.firstinspires.ftc.teamcode.localization.localizers.DriveEncoderAttributes;
import org.firstinspires.ftc.teamcode.localization.localizers.DriveEncoderLocalizer;

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
                                "left_front",
                                "right_front",
                                "left_back",
                                "right_back"
                        )
                )
                .localizer(() -> new DriveEncoderLocalizer(
                        hardwareMap,
                        new DriveEncoderAttributes(
                                "left_front",
                                "right_front",
                                "left_back",
                                "right_back",
                                Encoder.Direction.REVERSE,
                                Encoder.Direction.FORWARD,
                                Encoder.Direction.REVERSE,
                                Encoder.Direction.FORWARD,
                                16.5,21,
                                Odometer.createGobildaSwingArmPod()
                        )
                ))
                .velocityXNKFParams(new FinalLocalizerNKFConfig(1, 1, 1))
                .velocityYNKFParams(new FinalLocalizerNKFConfig(1, 1, 1))
                .velocityHeadingNKFParams(new FinalLocalizerNKFConfig(1, 1, 1))
                .accelerationXNKFParams(new FinalLocalizerNKFConfig(1, 1, 1))
                .accelerationYNKFParams(new FinalLocalizerNKFConfig(1, 1, 1))
                .accelerationHeadingNKFParams(new FinalLocalizerNKFConfig(1, 1, 1))
                .motionConstraints(
                        new MotionConstraints(
                                0, 0, 0, 0,
                                0, 0, 0, 0,
                                0, 0, 0, 0
                        )
                )
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

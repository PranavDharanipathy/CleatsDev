package org.firstinspires.ftc.teamcode.localization.localizers

import org.firstinspires.ftc.teamcode.localization.Encoder
import org.firstinspires.ftc.teamcode.localization.OdometryPod

data class ThreeDeadWheelAttributes(

    val leftEncoderName: String,
    val rightEncoderName: String,
    val strafeEncoderName: String,

    val leftPodY: Double,
    val rightPodY: Double,
    val strafePodX: Double,

    val leftDirection: Encoder.Direction,
    val rightDirection: Encoder.Direction,
    val strafeDirection: Encoder.Direction,

    val odometryPodData: OdometryPod
)
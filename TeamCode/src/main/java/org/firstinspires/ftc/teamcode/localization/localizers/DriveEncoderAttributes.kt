package org.firstinspires.ftc.teamcode.localization.localizers

import org.firstinspires.ftc.teamcode.localization.Encoder
import org.firstinspires.ftc.teamcode.localization.Odometer

data class DriveEncoderAttributes(

    val leftFrontName: String,
    val rightFrontName: String,
    val leftBackName: String,
    val rightBackName: String,

    val leftFrontDirection: Encoder.Direction,
    val rightFrontDirection: Encoder.Direction,
    val leftBackDirection: Encoder.Direction,
    val rightBackDirection: Encoder.Direction,

    val chassisWidth: Double,
    val chassisLength: Double,

    val odometerData: Odometer
)
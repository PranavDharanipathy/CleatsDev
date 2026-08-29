package org.firstinspires.ftc.teamcode.following

data class PrecisionModeThresholds(
    val entryPositionDistance: Double,
    val exitPositionDistance: Double,
    val entryVelocity: Double,
    val exitVelocity: Double,
    val entryHeadingError: Double,
    val exitHeadingError: Double,
    val entryAngularVelocity: Double,
    val exitAngularVelocity: Double
)
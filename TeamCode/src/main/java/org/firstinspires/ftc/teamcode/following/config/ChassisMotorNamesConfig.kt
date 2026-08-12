package org.firstinspires.ftc.teamcode.following.config

data class ChassisMotorNamesConfig(
    val leftFrontMotorDeviceName: String,
    val rightFrontMotorDeviceName: String,
    val leftBackMotorDeviceName: String,
    val rightBackMotorDeviceName: String
) {
    fun assemble(): Array<String> {
        return arrayOf(
            leftFrontMotorDeviceName,
            rightFrontMotorDeviceName,
            leftBackMotorDeviceName,
            rightBackMotorDeviceName
        )
    }
}
package org.firstinspires.ftc.teamcode.following.config

import com.qualcomm.robotcore.hardware.DcMotorSimple

data class ChassisMotorDirectionsConfig(
    val leftFrontMotorDirection: DcMotorSimple.Direction,
    val rightFrontMotorDirection: DcMotorSimple.Direction,
    val leftBackMotorDirection: DcMotorSimple.Direction,
    val rightBackMotorDirection: DcMotorSimple.Direction
) {
    fun assemble(): Array<DcMotorSimple.Direction> {
        return arrayOf(
            leftFrontMotorDirection,
            rightFrontMotorDirection,
            leftBackMotorDirection,
            rightBackMotorDirection
        )
    }
}

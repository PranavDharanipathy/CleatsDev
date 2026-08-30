package org.firstinspires.ftc.teamcode.following.chassis

data class MotionConstraints(
    val vmaxF: Double,
    val vmaxS: Double,
    val vmaxD: Double,
    val vmaxH: Double,

    val amaxF: Double,
    val amaxS: Double,
    val amaxD: Double,
    val amaxH: Double,

    val dmaxF: Double,
    val dmaxS: Double,
    val dmaxD: Double,
    val dmaxH: Double
) {
    fun makeMecanumProfile(): MecanumProfile {

        return MecanumProfile(
            vmaxF, vmaxS, vmaxD,
            amaxF, amaxS, amaxD,
            dmaxF, dmaxS, dmaxD
        )
    }

    fun getAntiSlipRampRate(): Double = amaxF / vmaxF

}

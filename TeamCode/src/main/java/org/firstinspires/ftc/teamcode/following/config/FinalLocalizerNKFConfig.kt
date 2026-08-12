package org.firstinspires.ftc.teamcode.following.config

data class FinalLocalizerNKFConfig(
    val q: Double,
    val r: Double,
    val outlierThresholdMultiplier: Double
) {
    fun assemble(): DoubleArray {
        return doubleArrayOf(q, r, outlierThresholdMultiplier)
    }
}
package org.firstinspires.ftc.teamcode.dribble

import java.util.function.BooleanSupplier

open class Delay(val seconds: Double) : Command() {

    private var until = 0.0

    companion object {

        @JvmStatic
        fun makeCancelable(seconds: Double, cancelWhen: BooleanSupplier): Delay {

            val delay = Delay(seconds)
            delay.setCancelWhen(cancelWhen)

            return delay
        }
    }

    /** @return seconds left to wait */
    fun getRemaining(): Double = (until - CommandScheduler.now()).coerceAtLeast(0.0)

    override fun start() {
        until = CommandScheduler.now() + seconds
    }

    override fun isDone(): Boolean = CommandScheduler.now() >= until
}

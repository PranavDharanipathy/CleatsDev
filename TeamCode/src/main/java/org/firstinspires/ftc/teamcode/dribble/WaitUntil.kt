package org.firstinspires.ftc.teamcode.dribble

import java.util.function.BooleanSupplier

open class WaitUntil(private val condition: BooleanSupplier) : Command() {

    companion object {

        @JvmStatic
        fun makeCancelable(condition: BooleanSupplier, cancelWhen: BooleanSupplier): WaitUntil {

            val wait = WaitUntil(condition)
            wait.setCancelWhen(cancelWhen)

            return wait
        }
    }

    override fun isDone(): Boolean = condition.asBoolean
}

package org.firstinspires.ftc.teamcode.dribble

import java.util.function.BooleanSupplier

open class While @JvmOverloads constructor(
    private val condition: BooleanSupplier,
    private val body: Runnable = Runnable {}
) : Command() {

    companion object {

        @JvmStatic
        fun makeCancelable(condition: BooleanSupplier, cancelWhen: BooleanSupplier): While {
            return makeCancelable(condition, Runnable {}, cancelWhen)
        }

        @JvmStatic
        fun makeCancelable(condition: BooleanSupplier, body: Runnable, cancelWhen: BooleanSupplier): While {

            val loop = While(condition, body)
            loop.setCancelWhen(cancelWhen)

            return loop
        }
    }

    override fun update() {
        body.run()
    }

    override fun isDone(): Boolean = !condition.asBoolean
}

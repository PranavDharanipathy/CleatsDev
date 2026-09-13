package org.firstinspires.ftc.teamcode.dribble

import java.util.function.BooleanSupplier

open class ParallelCommand(vararg commands: Command) : CommandGroup(*commands) {

    companion object {

        @JvmStatic
        fun makeCancelable(cancelWhen: BooleanSupplier, vararg commands: Command): ParallelCommand {

            val parallel = ParallelCommand(*commands)
            parallel.setCancelWhen(cancelWhen)

            return parallel
        }
    }

    override fun start() {
        startChildren()
    }

    override fun update() {

        for (i in children.indices) stepChild(i)
    }

    override fun isDone(): Boolean {

        for (done in finished) if (!done) return false

        return true
    }
}

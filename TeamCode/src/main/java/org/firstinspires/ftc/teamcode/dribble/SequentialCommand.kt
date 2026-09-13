package org.firstinspires.ftc.teamcode.dribble

import java.util.function.BooleanSupplier

open class SequentialCommand(vararg commands: Command) : CommandGroup(*commands) {

    private var at = 0

    companion object {

        @JvmStatic
        fun makeCancelable(cancelWhen: BooleanSupplier, vararg commands: Command): SequentialCommand {

            val sequence = SequentialCommand(*commands)
            sequence.setCancelWhen(cancelWhen)

            return sequence
        }
    }

    fun getIndex(): Int = at

    override fun start() {

        at = 0

        for (i in finished.indices) finished[i] = false

        startChild(0)
    }

    override fun update() {

        while (at < children.size && stepChild(at)) {

            if (++at >= children.size) return

            startChild(at)
        }
    }

    override fun isDone(): Boolean = at >= children.size

    override fun stop(interrupted: Boolean) {

        if (at < children.size) finish(at, interrupted)
    }
}
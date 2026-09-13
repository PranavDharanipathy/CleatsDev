package org.firstinspires.ftc.teamcode.dribble

import java.util.function.BooleanSupplier

open class DeadlineCommand(deadline: Command, vararg alongside: Command) : CommandGroup(deadline, *alongside) {

    companion object {

        @JvmStatic
        fun makeCancelable(cancelWhen: BooleanSupplier, deadline: Command, vararg alongside: Command): DeadlineCommand {

            val group = DeadlineCommand(deadline, *alongside)
            group.setCancelWhen(cancelWhen)

            return group
        }
    }

    override fun start() {
        startChildren()
    }

    override fun update() {

        for (i in children.indices) stepChild(i)
    }

    override fun isDone(): Boolean = finished[0]
}

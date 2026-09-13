package org.firstinspires.ftc.teamcode.dribble

import java.util.function.BooleanSupplier

open class InstantCommand(private val action: Runnable) : Command() {

    companion object {

        @JvmStatic
        fun makeCancelable(action: Runnable, cancelWhen: BooleanSupplier): InstantCommand {

            //command already true

            val command = InstantCommand(action)
            command.setCancelWhen(cancelWhen)

            return command
        }
    }

    override fun start() {
        if (!shouldCancel()) action.run()
    }

    override fun isDone(): Boolean = true
}
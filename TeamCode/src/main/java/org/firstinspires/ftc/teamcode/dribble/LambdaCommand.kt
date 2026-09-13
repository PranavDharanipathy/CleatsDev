package org.firstinspires.ftc.teamcode.dribble

import java.util.function.BooleanSupplier
import java.util.function.Consumer

open class LambdaCommand : Command() {

    private var onStart = Runnable {}
    private var onUpdate = Runnable {}
    private var onStop = Consumer<Boolean> {}
    private var finish = BooleanSupplier { false }

    companion object {

        @JvmStatic
        fun makeCancelable(cancelWhen: BooleanSupplier): LambdaCommand {

            val command = LambdaCommand()
            command.setCancelWhen(cancelWhen)

            return command
        }
    }

    fun setStart(onStart: Runnable): LambdaCommand {

        this.onStart = onStart
        return this
    }

    fun setUpdate(onUpdate: Runnable): LambdaCommand {

        this.onUpdate = onUpdate
        return this
    }

    /** @param onStop takes whether the command was interrupted rather than finishing */
    fun setStop(onStop: Consumer<Boolean>): LambdaCommand {

        this.onStop = onStop
        return this
    }

    fun setIsDone(finish: BooleanSupplier): LambdaCommand {

        this.finish = finish
        return this
    }

    override fun start() {
        onStart.run()
    }

    override fun update() {
        onUpdate.run()
    }

    override fun isDone(): Boolean = finish.asBoolean

    override fun stop(interrupted: Boolean) {
        onStop.accept(interrupted)
    }
}

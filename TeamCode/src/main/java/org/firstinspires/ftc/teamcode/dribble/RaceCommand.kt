package org.firstinspires.ftc.teamcode.dribble

import java.util.function.BooleanSupplier

open class RaceCommand(vararg commands: Command) : CommandGroup(*commands) {

    private var won = false

    companion object {

        @JvmStatic
        fun makeCancelable(cancelWhen: BooleanSupplier, vararg commands: Command): RaceCommand {

            val race = RaceCommand(*commands)
            race.setCancelWhen(cancelWhen)

            return race
        }
    }

    override fun start() {

        won = false
        startChildren()
    }

    override fun update() {

        for (i in children.indices) if (stepChild(i)) won = true
    }

    override fun isDone(): Boolean = won
}

package org.firstinspires.ftc.teamcode.dribble

import java.util.function.BooleanSupplier

abstract class Command {

    val requirements = ArrayList<Subsystem>()

    private var cancelCondition: BooleanSupplier? = null
    private var cancelRequested = false

    private var interruptible = true
    private var composed = false

    open fun start() {}

    open fun update() {}

    open fun isDone(): Boolean = false

    open fun stop(interrupted: Boolean) {}

    /** Claims exclusive control over a subsystem */
    fun requires(vararg subsystems: Subsystem): Command {

        requirements.addAll(subsystems)
        return this
    }

    fun setCancelWhen(condition: BooleanSupplier): Command {

        cancelCondition = condition
        return this
    }

    fun shouldCancel(): Boolean = cancelRequested || cancelCondition?.asBoolean == true

    /** Ends this command on the next loop */
    fun cancel() {

        cancelRequested = true
        CommandScheduler.cancel(this)
    }

    fun setInterruptible(value: Boolean): Command {

        interruptible = value
        return this
    }

    fun isInterruptible(): Boolean = interruptible

    fun isComposed(): Boolean = composed

    fun then(vararg next: Command): Command = SequentialCommand(this, *next)

    fun and(vararg others: Command): Command = ParallelCommand(this, *others)

    fun raceWith(vararg others: Command): Command = RaceCommand(this, *others)

    fun withDeadline(deadline: Command): Command = DeadlineCommand(deadline, this)

    fun asDeadlineFor(vararg others: Command): Command = DeadlineCommand(this, *others)

    fun withTimeout(seconds: Double): Command = RaceCommand(this, Delay(seconds))

    fun schedule(): Command {

        CommandScheduler.schedule(this)
        return this
    }

    fun isScheduled(): Boolean = CommandScheduler.isScheduled(this)

    fun begin() {

        cancelRequested = false
        start()
    }

    internal fun compose() {

        require(!composed) { "A command cannot be in two groups at once, make a second one instead!" }

        composed = true
    }
}

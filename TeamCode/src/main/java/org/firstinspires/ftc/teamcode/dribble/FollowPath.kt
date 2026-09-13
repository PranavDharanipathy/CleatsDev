package org.firstinspires.ftc.teamcode.dribble

import org.firstinspires.ftc.teamcode.path.Maneuver
import org.firstinspires.ftc.teamcode.path.Movement

import java.util.function.BooleanSupplier

open class FollowPath(
    private val dribblePathController: DribblePathController,
    val maneuver: Maneuver,
    private val precisionStop: Boolean
) : Command() {

    constructor(movement: Movement) : this(DribblePathController.getActive(), Maneuver().addMovement(movement), true)

    constructor(maneuver: Maneuver) : this(DribblePathController.getActive(), maneuver, true)

    constructor(dribblePathController: DribblePathController, movement: Movement) : this(dribblePathController, Maneuver().addMovement(movement), true)

    constructor(dribblePathController: DribblePathController, maneuver: Maneuver) : this(dribblePathController, maneuver, true)

    init {
        requires(dribblePathController)
    }

    companion object {

        @JvmStatic
        fun makeCancelable(movement: Movement, cancelWhen: BooleanSupplier): FollowPath =
            makeCancelable(DribblePathController.getActive(), Maneuver().addMovement(movement), true, cancelWhen)

        @JvmStatic
        fun makeCancelable(maneuver: Maneuver, cancelWhen: BooleanSupplier): FollowPath =
            makeCancelable(DribblePathController.getActive(), maneuver, true, cancelWhen)

        @JvmStatic
        fun makeCancelable(dribblePathController: DribblePathController, movement: Movement, cancelWhen: BooleanSupplier): FollowPath =
            makeCancelable(dribblePathController, Maneuver().addMovement(movement), true, cancelWhen)

        @JvmStatic
        fun makeCancelable(dribblePathController: DribblePathController, maneuver: Maneuver, cancelWhen: BooleanSupplier): FollowPath =
            makeCancelable(dribblePathController, maneuver, true, cancelWhen)

        @JvmStatic
        fun makeCancelable(dribblePathController: DribblePathController, maneuver: Maneuver, precisionStop: Boolean, cancelWhen: BooleanSupplier): FollowPath {

            val command = FollowPath(dribblePathController, maneuver, precisionStop)
            command.setCancelWhen(cancelWhen)

            return command
        }
    }

    /** @return how much of the path is done, 0 to 1 */
    fun getPercent(): Double = maneuver.percent

    override fun start() {
        dribblePathController.pathController.follow(maneuver, precisionStop)
    }

    override fun isDone(): Boolean = !dribblePathController.pathController.isFollowing

    override fun stop(interrupted: Boolean) {

        if (interrupted) dribblePathController.pathController.cancel()
    }
}

package org.firstinspires.ftc.teamcode.dribble

import org.firstinspires.ftc.teamcode.util.Pose

import java.util.function.BooleanSupplier

open class DriveTo(
    private val dribblePathController: DribblePathController,
    private val dribblePathOptimizer: DribblePathOptimizer,
    val target: Pose,
    private val precisionStop: Boolean
) : Command() {

    private var planned = false
    private var planMillis = 0.0

    constructor(target: Pose) : this(DribblePathController.getActive(), DribblePathOptimizer.getActive(), target, true)

    constructor(dribblePathController: DribblePathController, dribblePathOptimizer: DribblePathOptimizer, target: Pose) : this(dribblePathController, dribblePathOptimizer, target, true)

    init {
        requires(dribblePathController)
    }

    companion object {

        @JvmStatic
        fun makeCancelable(target: Pose, cancelWhen: BooleanSupplier): DriveTo =
            makeCancelable(DribblePathController.getActive(), DribblePathOptimizer.getActive(), target, true, cancelWhen)

        @JvmStatic
        fun makeCancelable(dribblePathController: DribblePathController, dribblePathOptimizer: DribblePathOptimizer, target: Pose, cancelWhen: BooleanSupplier): DriveTo =
            makeCancelable(dribblePathController, dribblePathOptimizer, target, true, cancelWhen)

        @JvmStatic
        fun makeCancelable(dribblePathController: DribblePathController, dribblePathOptimizer: DribblePathOptimizer, target: Pose, precisionStop: Boolean, cancelWhen: BooleanSupplier): DriveTo {

            val command = DriveTo(dribblePathController, dribblePathOptimizer, target, precisionStop)
            command.setCancelWhen(cancelWhen)

            return command
        }
    }

    /** @return whether a way through was found */
    fun wasPlanned(): Boolean = planned

    /** @return how long the planning took in milliseconds */
    fun getPlanMillis(): Double = planMillis

    override fun start() {

        val began = System.nanoTime()

        val path = dribblePathOptimizer.pathFrom(dribblePathController.pathController.pose, target)

        planMillis = (System.nanoTime() - began) / 1e6
        planned = path != null

        if (path != null) dribblePathController.pathController.follow(path, precisionStop)
    }

    override fun isDone(): Boolean = !planned || !dribblePathController.pathController.isFollowing

    override fun stop(interrupted: Boolean) {

        if (interrupted) dribblePathController.pathController.cancel()
    }
}

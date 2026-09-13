package org.firstinspires.ftc.teamcode.dribble

abstract class CommandGroup(vararg children: Command) : Command() {

    val children: Array<out Command> = children

    protected val finished = BooleanArray(children.size)

    init {

        require(children.isNotEmpty()) { "A command group needs at least one command in it!" }

        for (child in children) {

            child.compose()

            for (subsystem in child.requirements) {

                if (requirements.contains(subsystem)) continue

                requirements.add(subsystem)
            }
        }
    }

    protected fun startChild(i: Int) {

        finished[i] = false
        children[i].begin()
    }

    protected fun startChildren() {

        for (i in children.indices) startChild(i)
    }

    /** @return whether the child ended by finishing or by cancellation */
    protected fun stepChild(i: Int): Boolean {

        if (finished[i]) return false

        if (children[i].shouldCancel()) {

            finish(i, true)
            return true
        }

        children[i].update()

        if (!children[i].isDone()) return false

        finish(i, false)
        return true
    }

    protected fun finish(i: Int, interrupted: Boolean) {

        if (finished[i]) return

        finished[i] = true
        children[i].stop(interrupted)
    }

    override fun stop(interrupted: Boolean) {

        for (i in children.indices) finish(i, true)
    }
}

package org.firstinspires.ftc.teamcode.util

object BinarySearch {

    fun firstGreaterOrEqual(values: DoubleArray, target: Double): Int {

        var low = 0
        var high = values.lastIndex

        while (low < high) {
            val mid = (low + high) ushr 1
            if (values[mid] >= target) high = mid
            else low = mid + 1
        }

        return low
    }
}
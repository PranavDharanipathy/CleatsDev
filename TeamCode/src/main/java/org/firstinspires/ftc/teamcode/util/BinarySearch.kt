package org.firstinspires.ftc.teamcode.util;

public class BinarySearch {

    private BinarySearch() {}

    /**
     * Returns the index of the first value >= target.
     * Assumes values are sorted in ascending order.
     */
    public static int firstGreaterOrEqual(double[] values, double target) {

        int low = 0;
        int high = values.length - 1;

        while (low < high) {
            int mid = (low + high) >>> 1;

            if (values[mid] >= target)
                high = mid;
            else
                low = mid + 1;
        }

        return low;
    }
}

package org.firstinspires.ftc.teamcode.util;

public class MiniHeap {

    private int[] node;
    private double[] key;
    private int size;

    public MiniHeap(int capacity) {

        node = new int[Math.max(16, capacity)];
        key = new double[node.length];
    }

    public boolean isEmpty() {
        return size == 0;
    }

    public void push(int value, double priority) {

        if (size == node.length) {

            node = java.util.Arrays.copyOf(node, size * 2);
            key = java.util.Arrays.copyOf(key, size * 2);
        }

        int i = size++;
        node[i] = value;
        key[i] = priority;

        while (i > 0) {

            int parent = (i - 1) >>> 1;
            if (key[parent] <= key[i]) break;

            swap(parent, i);
            i = parent;
        }
    }

    public int pop() {

        if (size == 0) throw new IllegalStateException("There is nothing left in this heap to pop!");

        int top = node[0];

        size--;
        node[0] = node[size];
        key[0] = key[size];

        int i = 0;

        while (true) {

            int left = 2 * i + 1, right = left + 1, smallest = i;

            if (left < size && key[left] < key[smallest]) smallest = left;
            if (right < size && key[right] < key[smallest]) smallest = right;
            if (smallest == i) break;

            swap(smallest, i);
            i = smallest;
        }

        return top;
    }

    private void swap(int a, int b) {

        int n = node[a]; node[a] = node[b]; node[b] = n;
        double k = key[a]; key[a] = key[b]; key[b] = k;
    }
}

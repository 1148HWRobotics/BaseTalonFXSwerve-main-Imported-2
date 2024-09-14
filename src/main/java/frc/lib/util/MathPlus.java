package frc.lib.util;

public class MathPlus {
    public static double clampAbsVal(double val, double max) {
        max = Math.abs(max);
        if (val > max)
            return max;
        if (val < -max)
            return -max;
        return val;
    }

    public static double clampVal(double val, double min, double max) {
        if (val > max)
            return max;
        if (val < min)
            return min;
        return val;
    }

    public static int boolFac(boolean b) {
        return b ? 1 : -1;
    }

    public static boolean withinBounds(double v, double high, double low) {
        return v < high && v > low;
    }
}

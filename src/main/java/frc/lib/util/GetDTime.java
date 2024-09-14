package frc.lib.util;

import frc.Core.Time;

public class GetDTime {
    double lastTime = Double.MIN_VALUE;

    public double tick() {
        var temp = lastTime;
        lastTime = Time.getTimeSincePower();
        return lastTime - temp;
    }
}

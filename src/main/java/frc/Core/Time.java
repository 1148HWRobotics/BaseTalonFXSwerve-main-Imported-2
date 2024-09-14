package frc.Core;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.Lambda;
import frc.lib.util.Promise;

public class Time {
    public static double getTimeSincePower() {
        return Timer.getFPGATimestamp();
    }

    public static double getMatchTime() {
        return Timer.getMatchTime();
    }

    public static void timeout(Lambda run, double seconds) {
        var prom = Promise.timeout(seconds);
        prom.then(run);
    }
}

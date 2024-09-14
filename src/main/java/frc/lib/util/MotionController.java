package frc.lib.util;

public interface MotionController {
    public double solve(double error, double dTime);

    public MotionController clone();
}

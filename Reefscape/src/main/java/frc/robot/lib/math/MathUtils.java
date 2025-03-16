package frc.robot.lib.math;

public class MathUtils {
    public static double clamp(double num, double upperBound, double lowerBound){
        return Math.min(Math.max(num, lowerBound), upperBound);
    }
}

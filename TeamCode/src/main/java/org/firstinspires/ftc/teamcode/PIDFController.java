package org.firstinspires.ftc.teamcode;

/**
 * Simple PIDF Controller for velocity control
 * Used for flywheel/shooter tuning with kV and kS feedforward
 */
public class PIDFController {
    private double kP, kI, kD, kF;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.lastTime = System.nanoTime();
    }

    /**
     * Update PIDF coefficients (for dynamic tuning)
     */
    public void setPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Calculate control output based on error
     * @param error (target - current)
     * @return motor power [-1.0, 1.0]
     */
    public double calculate(double error) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9; // Convert to seconds
        lastTime = currentTime;

        // Proportional
        double p = kP * error;

        // Integral (with anti-windup)
        integralSum += error * dt;
        // Clamp integral to prevent windup
        integralSum = Math.max(-1.0, Math.min(1.0, integralSum));
        double i = kI * integralSum;

        // Derivative
        double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;
        double d = kD * derivative;
        lastError = error;

        // Feedforward (kF is set externally via setPIDF)
        double output = p + i + d + kF;

        // Clamp output to valid motor power range
        return Math.max(-1.0, Math.min(1.0, output));
    }

    /**
     * Reset integral and derivative state
     */
    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        lastTime = System.nanoTime();
    }
}


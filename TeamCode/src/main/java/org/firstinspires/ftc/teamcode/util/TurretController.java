package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretController {

    // POSITION LOOP (Outer)
    private double pPos, iPos, dPos;
    public static final double maxTargetVelocity = 0;

    // VELOCITY LOOP (Inner)
    private double pVel, iVel, dVel;
    private double kV;

    // Integral windup limits
    private double posIntegralLimit = Double.MAX_VALUE;
    private double velIntegralLimit = Double.MAX_VALUE;

    // STATE VARIABLES
    private double posIntegral = 0;
    private double lastPosMeasurement = Double.NaN;

    private double velIntegral = 0;
    private double lastVelMeasurement = Double.NaN;

    // Telemetry variables
    private double lastTargetVelocity = 0;
    private double lastOutput = 0;
    private double lastPosErrorOut = 0;
    private double lastVelErrorOut = 0;

    private ElapsedTime timer = new ElapsedTime();

    public TurretController(double pPos, double iPos, double dPos,
                            double pVel, double iVel, double dVel, double kV) {
        this.pPos = pPos; this.iPos = iPos; this.dPos = dPos;
        this.pVel = pVel; this.iVel = iVel; this.dVel = dVel;
        this.kV = kV;
        timer.reset();
    }

    /**
     * CALCULATION LOOP
     * @param targetAngleDeg  Target Angle in Degrees
     * @param currentAngleDeg Current Angle in Degrees
     * @param currentVelDegPerSec Current Velocity in Degrees/s
     */
    public double calculate(double targetAngleDeg, double currentAngleDeg, double currentVelDegPerSec) {

        double dt = timer.seconds();
        timer.reset();
        if (dt > 0.2 || dt <= 0) dt = 0.01;

        // --- 1. OUTER LOOP (POSITION) ---
        double posError = targetAngleDeg - currentAngleDeg;

        posIntegral += posError * dt;
        posIntegral = clamp(posIntegral, -posIntegralLimit, posIntegralLimit);


        double posDerivative = 0;
        if (!Double.isNaN(lastPosMeasurement)) {
            posDerivative = -(currentAngleDeg - lastPosMeasurement) / dt;
        }
        lastPosMeasurement = currentAngleDeg;

        double targetVelocity = (pPos * posError) + (iPos * posIntegral) + (dPos * posDerivative);

        // Clamp Velocity
        if (Math.abs(targetVelocity) > maxTargetVelocity) {
            targetVelocity = Math.signum(targetVelocity) * maxTargetVelocity;
        }

        // 2. INNER LOOP (VELOCITY)
        double velError = targetVelocity - currentVelDegPerSec;

        velIntegral += velError * dt;
        velIntegral = clamp(velIntegral, -velIntegralLimit, velIntegralLimit);

        // Derivative on measurement (avoids spikes on target velocity changes)
        double velDerivative = 0;
        if (!Double.isNaN(lastVelMeasurement)) {
            velDerivative = -(currentVelDegPerSec - lastVelMeasurement) / dt;
        }
        lastVelMeasurement = currentVelDegPerSec;

        // Calculate Motor Power
        double output = (pVel * velError) + (iVel * velIntegral) + (dVel * velDerivative) + (kV * targetVelocity);

        // telemetry values
        lastTargetVelocity = targetVelocity;
        lastPosErrorOut = posError;
        lastVelErrorOut = velError;


        // Clamp Motor Power
        lastOutput = clamp(output, -1.0, 1.0);

        return lastOutput;
    }

    // Integral windup limits

    public void setPosIntegralLimit(double limit) {
        this.posIntegralLimit = Math.abs(limit);
    }

    public void setVelIntegralLimit(double limit) {
        this.velIntegralLimit = Math.abs(limit);
    }

    // Coefficient setter methods

    public void setCoefficients(double pPos, double iPos, double dPos,
                                double pVel, double iVel, double dVel, double kF) {
        setPosCoefficients(pPos, iPos, dPos);
        setVelCoefficients(pVel, iVel, dVel);
        setFeedForward(kF);
    }

    public void setPosCoefficients(double pPos, double iPos, double dPos) {
        this.pPos = pPos; this.iPos = iPos; this.dPos = dPos;
    }

    public void setVelCoefficients(double pVel, double iVel, double dVel) {
        this.pVel = pVel; this.iVel = iVel; this.dVel = dVel;
    }

    public void setFeedForward(double kF){
        this.kV = kF;
    }


    public double getPositionError() { return lastPosErrorOut; }
    public double getVelocityError() { return lastVelErrorOut; }
    public double getTargetVelocity() { return lastTargetVelocity; }
    public double getOutput() { return lastOutput; }
    public double getPosIntegral() { return posIntegral; }
    public double getVelIntegral() { return velIntegral; }

    /**
     * Resets all accumulated state.
     */
    public void reset() {
        posIntegral = 0;
        velIntegral = 0;
        lastPosMeasurement = Double.NaN;
        lastVelMeasurement = Double.NaN;
        lastTargetVelocity = 0;
        lastOutput = 0;
        lastPosErrorOut = 0;
        lastVelErrorOut = 0;
        timer.reset();
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

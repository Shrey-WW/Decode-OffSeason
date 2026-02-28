package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.util.MathUtils;

public class TurretController {

    // POSITION LOOP (Outer)
    private double pPos, iPos, dPos;
    public static final double maxTargetVelocity = 150;
    public static final double maxPosition = 120;
    public static final double minPosition = -90;
    // VELOCITY LOOP (Inner)
    private double pVel, iVel, dVel;
    private double kV;
    private double robotVelocityFF = 0;

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
    private double lastPosError = 0;
    private double lastVelError = 0;

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
        targetAngleDeg = MathUtils.clamp(targetAngleDeg, minPosition, maxPosition);

        double dt = timer.seconds();
        timer.reset();
        if (dt > 0.2 || dt <= 0) dt = 0.01;

        //          OUTER LOOP (POSITION)
        double posError = targetAngleDeg - currentAngleDeg;

        posIntegral += posError * dt;


        double posDerivative = 0;
        if (!Double.isNaN(lastPosMeasurement)) {
            posDerivative = -(currentAngleDeg - lastPosMeasurement) / dt;
        }
        lastPosMeasurement = currentAngleDeg;

        double targetVelocity = (pPos * posError) + (iPos * posIntegral) + (dPos * posDerivative);

        // Clamp Velocity
        targetVelocity = MathUtils.clamp(targetVelocity, -maxTargetVelocity, maxTargetVelocity);

        //          INNER LOOP (VELOCITY)
        double velError = targetVelocity - currentVelDegPerSec;

        velIntegral += velError * dt;

        double velDerivative = 0;
        if (!Double.isNaN(lastVelMeasurement)) {
            velDerivative = -(currentVelDegPerSec - lastVelMeasurement) / dt;
        }
        lastVelMeasurement = currentVelDegPerSec;

        // Calculate Motor Power
        double output = (pVel * velError) + (iVel * velIntegral) + (dVel * velDerivative) + (kV * targetVelocity);

        // telemetry values
        lastTargetVelocity = targetVelocity;
        lastPosError = posError;
        lastVelError = velError;


        // Clamp Motor Power
        lastOutput = MathUtils.clamp(output, -1.0, 1.0);

        return lastOutput;
    }

    /**
     * CALCULATION LOOP
     * @param targetAngleDeg  Target Angle in Degrees
     * @param currentAngleDeg Current Angle in Degrees
     * @param currentVelDegPerSec Current Velocity in Degrees/s
     * @param robotAngularVel Robot Angular Velocity in Degrees
     */
    public double calculate(double targetAngleDeg, double currentAngleDeg, double currentVelDegPerSec, double robotAngularVel){
        robotVelocityFF = robotAngularVel;
        return calculate(targetAngleDeg, currentAngleDeg, currentVelDegPerSec);
    }

    /**
     * Only runs the inner velocity loop
     * @param targetVelDegPerSec Target velocity in degrees/s
     * @param currentVelDegPerSec Current velocity in degrees/s
     */
    public double calculateVelocityOnly(double targetVelDegPerSec, double currentVelDegPerSec) {
        double dt = timer.seconds();
        timer.reset();
        if (dt > 0.2 || dt <= 0) dt = 0.01;

        double velError = targetVelDegPerSec - currentVelDegPerSec;

        velIntegral += velError * dt;
        velIntegral = MathUtils.clamp(velIntegral, -velIntegralLimit, velIntegralLimit);

        double velDerivative = 0;
        if (!Double.isNaN(lastVelMeasurement)) {
            velDerivative = -(currentVelDegPerSec - lastVelMeasurement) / dt;
        }
        lastVelMeasurement = currentVelDegPerSec;

        double output = (pVel * velError) + (iVel * velIntegral) + (dVel * velDerivative) + (kV * targetVelDegPerSec);

        lastTargetVelocity = targetVelDegPerSec;
        lastVelError = velError;
        lastOutput = MathUtils.clamp(output, -1.0, 1.0);

        return lastOutput;
    }

    // Integral windup limits

    public void setPosIntegralLimit(double limit) {
        posIntegralLimit = Math.abs(limit);
    }

    public void setVelIntegralLimit(double limit) {
        velIntegralLimit = Math.abs(limit);
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


    public double getPositionError() { return lastPosError; }
    public double getVelocityError() { return lastVelError; }
    public double getTargetVelocity() { return lastTargetVelocity; }
    public double getLastOutput() { return lastOutput; }

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
        lastPosError = 0;
        lastVelError = 0;
        timer.reset();
    }
}

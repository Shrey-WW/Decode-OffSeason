package org.firstinspires.ftc.teamcode.util;

public class TurretKalmanFilter {

    private double Q;       // process noise — how fast the true target can change
    private double R_odom;  // odometry measurement noise
    private double R_ll;    // limelight measurement noise

    private double x;       // state estimate (turret target, degrees)
    private double P;       // error covariance

    public TurretKalmanFilter(double q, double ROdom, double RLL, double initialEstimate) {
        Q = q;
        R_odom = ROdom;
        R_ll = RLL;
        x = initialEstimate;
        P = ROdom;
    }

    /**
     * Run one filter cycle.
     * @param odomTarget  odometry-based turret target (degrees), always provided
     * @param llTarget    limelight-based turret target (degrees), null if no valid result
     * @return fused turret target (degrees)
     */
    public double update(double odomTarget, Double llTarget) {
        // 1. Predict — covariance grows between measurements
        P += Q;

        // 2. Update with odometry
        double K_odom = P / (P + R_odom);
        x += K_odom * (odomTarget - x);
        P = (1 - K_odom) * P;

        // 3. Update with limelight (only when valid)
        if (llTarget != null) {
            double K_ll = P / (P + R_ll);
            x += K_ll * (llTarget - x);
            P = (1 - K_ll) * P;
        }

        return x;
    }

    public void setQ(double Q)          { this.Q = Q; }
    public void setROdom(double R_odom) { this.R_odom = R_odom; }
    public void setRLl(double R_ll)     { this.R_ll = R_ll; }
    public double getEstimate()         { return x; }
    public double getCovariance()       { return P; }
}
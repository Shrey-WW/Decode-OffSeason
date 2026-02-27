package org.firstinspires.ftc.teamcode.util;

public class TurretKalmanFilter {

    private double Q;       // process noise — how fast the true target can change
    private double R_odom;  // odometry measurement noise
    private double R_ll;    // limelight measurement noise

    private double estimate;       // state estimate (turret target, degrees)
    private double P;       // error covariance

    public TurretKalmanFilter(double q, double ROdom, double RLL, double initialEstimate) {
        Q = q;
        R_odom = ROdom;
        R_ll = RLL;
        estimate = initialEstimate;
        P = ROdom;
    }

    /**
     * Run one filter cycle.
     * @param odomTarget  odometry turret target (Degrees)
     * @param llTarget    limelight turret target (Degrees)
     * @return fused turret target (degrees)
     */
    public double estimate(double odomTarget, Double llTarget) {
        P += Q;

        //  Odom prediction
        double K_odom = P / (P + R_odom);
        estimate += K_odom * (odomTarget - estimate);
        P = (1 - K_odom) * P;

        //  Limelight prediction
        if (llTarget != null) {
            double K_ll = P / (P + R_ll);
            estimate += K_ll * (llTarget - estimate);
            P = (1 - K_ll) * P;
        }

        return estimate;
    }

    public void setQ(double Q)          { this.Q = Q; }
    public void setROdom(double R_odom) { this.R_odom = R_odom; }
    public void setRLl(double R_ll)     { this.R_ll = R_ll; }
    public double getEstimate()         { return estimate; }
    public double getCovariance()       { return P; }
}
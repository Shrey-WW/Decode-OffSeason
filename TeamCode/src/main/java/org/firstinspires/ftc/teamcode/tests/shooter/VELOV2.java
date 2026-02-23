package org.firstinspires.ftc.teamcode.tests.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/**
 * Tuning modes:
 *   0 - Manual:  set flywheel velocity via velo, shows distance + LUT suggestion
 *   1 - Test:    full VELO logic (LUT picks velocity, controls transfer + intake)
 */
@Config
@TeleOp(group = "tests")
public class VELOV2 extends CommandOpMode {

    // 0 = Manual | 1 = Test
    public static int Mode = 0;

    // Manual target velocity
    public static double velo = 0;

    private static final double HEIGHT_LIMELIGHT      = 16.5;
    private static final double LIMELIGHT_MOUNT_ANGLE = 12.68;
    private static final double HEIGHT_OF_APRILTAG    = 29.5;
    private static final double DISTANCE_FILTER_ALPHA = 0.30;
    private static final String[] MODE_NAMES          = {"Manual", "Test"};

    private Shooter shooter;
    private Transfer transfer;
    private Intake intake;
    private Limelight3A limelight;

    private final InterpLUT VELO_LUT = new InterpLUT();
    private double filteredDistance = Double.NaN;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VELO_LUT.add(0,   800);
        VELO_LUT.add(24,  1000);
        VELO_LUT.add(50,  1100);
        VELO_LUT.add(67,  1220);
        VELO_LUT.add(90,  1540);
        VELO_LUT.add(98,  1560);
        VELO_LUT.add(110, 1700);
        VELO_LUT.add(150, 1900);
        VELO_LUT.createLUT();

        shooter  = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake   = new Intake(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        shooter.moveServo(.8);
        limelight.start();
    }

    @Override
    public void run() {
        super.run();
        LLResult llResult = limelight.getLatestResult();

        telemetry.addData("Mode",             MODE_NAMES[Mode]);
        telemetry.addData("Shooter Velocity", shooter.getVelo());
        telemetry.addLine();

        switch (Mode) {
            case 0: // Manual — set velo
                shooter.setVelocity(velo);
                transfer.Spin(1);
                intake.Spin(1);
                telemetry.addData("Manual Velo", velo);
                if (llResult != null && llResult.isValid()) {
                    double dist = getDistanceFromTag(llResult);
                    telemetry.addData("Distance (in)",   dist);
                    telemetry.addLine();
                    telemetry.addData("Tx", llResult.getTx());
                    telemetry.addData("Ty", llResult.getTy());
                    telemetry.addData("Ta", llResult.getTa());
                }
                break;

            case 1: // Test — full VELO logic
                if (llResult != null && llResult.isValid()) {
                    double targetVelo  = VELO_LUT.get(getDistanceFromTag(llResult));
                    double currentVelo = shooter.getVelo();
                    shooter.setVelocity(targetVelo);
                    if (currentVelo > targetVelo - 80) {
                        transfer.Spin(1);
                        intake.Spin(1);
                    } else {
                        transfer.Spin(.4);
                        intake.Spin(0);
                    }
                    telemetry.addData("Distance (in)",   getDistanceFromTag(llResult));
                    telemetry.addData("Target Velocity", targetVelo);
                    telemetry.addData("Velocity Error",  targetVelo - currentVelo);
                }
        }

        telemetry.update();
    }

    private double getDistanceFromTag(LLResult lr) {
        double raw = (HEIGHT_OF_APRILTAG - HEIGHT_LIMELIGHT) /
                (Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE + lr.getTy())) * Math.cos(Math.toRadians(lr.getTx())));
        if (Double.isNaN(filteredDistance)) {
            filteredDistance = raw;
        } else {
            filteredDistance = DISTANCE_FILTER_ALPHA * raw + (1 - DISTANCE_FILTER_ALPHA) * filteredDistance;
        }
        return filteredDistance;
    }
}
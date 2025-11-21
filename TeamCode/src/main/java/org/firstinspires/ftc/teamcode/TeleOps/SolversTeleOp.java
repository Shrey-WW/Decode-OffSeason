package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class SolversTeleOp extends OpMode {
    public static ElapsedTime timer = new ElapsedTime();

    Robot bot;

    private GamepadEx driver;

    private Limelight3A limelight;
    private IMU imu;
    private static final double TICKS_PER_REV = 2403.125;
    private static final double unwrapThreshold = 2200;
    private static final double bearingMin = 1.5;

    private long lastLoop = System.nanoTime();
    private double lastHeading = 0;

    private boolean isUnwrapping = false;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        bot = new Robot(this);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        driver = new GamepadEx(gamepad1);

        // Bind commands:
        new GamepadButton(driver, GamepadKeys.Button.right_trigger)
                .whenPressed(new IntakeCommand(robot.Intake));

        new GamepadButton(driver, GamepadKeys.Button.X)
                .toggleWhenPressed(new SlowModeCommand(robot.drive));
    }

    @Override
    public void onStartButtonPressed(){
        Turret.X.velPID();
        limelight.start();
        bot.drive.schedule();
    }

    @Override
    public void onWaitForStart() {
        Turret.X.resetPwr();
        Turret.X.PIDReset();
    }

    @Override
    public void loop(){
        driver.readButtons();
        button.readValue();
        toggle.readValue();
    }
}

//package org.firstinspires.ftc.teamcode.Autos;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
//
//import org.firstinspires.ftc.teamcode.R;
//import org.firstinspires.ftc.teamcode.Requirements.BluePaths;
//import org.firstinspires.ftc.teamcode.Requirements.RedPaths;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
//import org.firstinspires.ftc.teamcode.Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import java.util.ArrayList;
//
//@Autonomous
//public class Auto extends CommandOpMode {
//    private Follower follower;
//    Limelight3A limelight;
//    IMU imu;
//    BluePaths Paths;
//    RedPaths redPaths;
//    Transfer transfer;
//    Shooter shooter;
//    Intake intake;
//    Turret turret;
//    InstantCommand startIntake, stopIntake, reverseIntake, hoodUp;
//    SequentialCommandGroup AutoSequence, AutoSequence2, AutoSequence3, AutoSequence4;
//    ArrayList<String> arrayList;
//
//    @Override
//    public void initialize() {
//        super.reset();
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//        imu = hardwareMap.get(IMU.class, "imu");
//        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP);
//        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(19, 123, Math.toRadians(143.5)));
//        transfer = new Transfer(hardwareMap);
//        shooter = new Shooter(hardwareMap);
//        intake = new Intake(hardwareMap);
//        turret = new Turret(hardwareMap);
//        telemetry.addLine("X for red/ O for blue");
//        telemetry.update();
//        if (gamepad1.a) {
//            arrayList.add("Red");
//        } else if (gamepad1.b) {
//            arrayList.add("Blue");
//        }
//        telemetry.addLine("X for Close/ O for Far");
//        telemetry.update();
//        if (gamepad1.a) {
//            arrayList.add("Close");
//        } else if (gamepad1.b) {
//            arrayList.add("Far");
//        }
//        telemetry.addLine("X for 9/ O for 12");
//        telemetry.update();
//        if (gamepad1.a) {
//            arrayList.add("6");
//        } else if (gamepad1.b) {
//            arrayList.add("9");
//        }
//        if (arrayList.get(0).equals("Blue")) {
//            if (arrayList.get(1).equals("Close")) {
//                if (arrayList.get(2).equals("9")) {
//                    Paths = new BluePaths(BluePaths.AutoType.CLOSE_NINE, follower);
//
//                } else if (arrayList.get(2).equals("12")) {
//                    Paths = new BluePaths(BluePaths.AutoType.CLOSE_TWELVE_NO_TURRET, follower);
//
//                }
//            } else if (arrayList.get(1).equals("Far")) {
//                if (arrayList.get(2).equals("9")) {
//                    Paths = new BluePaths(BluePaths.AutoType.FAR_NINE, follower);
//
//                } else if (arrayList.get(2).equals("12")) {
//                    Paths = new BluePaths(BluePaths.AutoType.FAR_TWELVE, follower);
//                }
//            }
//        }
//        if (arrayList.get(0).equals("Red")) {
//            if (arrayList.get(1).equals("Close")) {
//                if (arrayList.get(2).equals("9")) {
//                    redPaths = new RedPaths(RedPaths.AutoType.CLOSE_NINE, follower);
//
//                } else if (arrayList.get(2).equals("12")) {
//                    redPaths = new RedPaths(RedPaths.AutoType.CLOSE_TWELVE, follower);
//
//                }
//            } else if (arrayList.get(1).equals("Far")) {
//                if (arrayList.get(2).equals("9")) {
//                    redPaths = new RedPaths(RedPaths.AutoType.FAR_NINE, follower);
//
//                } else if (arrayList.get(2).equals("12")) {
//                    redPaths = new RedPaths(RedPaths.AutoType.FAR_TWELVE, follower);
//                }
//            }
//            if(arrayList.get(0).equals("Blue")) {
//                Paths.buildPaths();
//            } else if (arrayList.get(0).equals("Red")) {
//                redPaths.buildPaths();
//            }
//            startIntake = new InstantCommand(() -> intake.Spin(1));
//            stopIntake = new InstantCommand(intake::PwrOff);
//            reverseIntake = new InstantCommand(() -> intake.Spin(-1));
//            hoodUp = new InstantCommand(() -> shooter.moveServo(0));
//            AutoSequence = new SequentialCommandGroup(
//                    new FollowPathCommand(follower, Paths.ShootPreloads),
//                    new FollowPathCommand(follower, Paths.Intake1)
//                    );
//            AutoSequence2 = new SequentialCommandGroup(
//                    new FollowPathCommand(follower, Paths.openGate)
//            );
//
//            AutoSequence3 = new SequentialCommandGroup(
//                    new FollowPathCommand(follower, Paths.goToScore1),
//                    new FollowPathCommand(follower, Paths.Intake2),
//                    new FollowPathCommand(follower, Paths.goToScore2)
//            );
//            AutoSequence4 = new SequentialCommandGroup(
//
//                    new FollowPathCommand(follower, Paths.Intake3),
//                    new FollowPathCommand(follower, Paths.goToScore3)
//            );
//            schedule(AutoSequence);
//            if (arrayList.get(1).equals("Close"))
//            {
//                schedule(AutoSequence2);
//            }
//            schedule(AutoSequence3);
//            if (arrayList.get(1).equals("12")) {
//                schedule(AutoSequence4);
//            }
//
//
//        }
//
//    }
//    @Override
//    public void run(){
//        follower.update();
//        super.run();
//    }
//}
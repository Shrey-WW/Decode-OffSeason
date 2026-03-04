package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.constants.AutoType;
import org.firstinspires.ftc.teamcode.subsystems.Rusty;
import org.firstinspires.ftc.teamcode.constants.Alliance;

@Config
@TeleOp (name = "MainTeleOp", group = "a teleop")
public class MainTeleOp extends CommandOpMode {
    Rusty rusty;
    private boolean allianceSelect = false;
    private boolean EndingPosition = false;
    Alliance alliance;
    AutoType autoType;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad1.setLedColor(0.721568627, .3, 0.1, Gamepad.LED_DURATION_CONTINUOUS);
    }

    @Override
    public void initialize_loop(){
        if (!allianceSelect){
            telemetry.addLine("What alliance?\nCROSS - BLUE\nTRIANGLE - RED");
            telemetry.update();
            if (gamepad1.a){
                alliance = Alliance.BLUE;
                allianceSelect = true;
            }
            if (gamepad1.y){
                alliance = Alliance.RED;
                allianceSelect = true;
            }
        }
        else if (allianceSelect && !EndingPosition) {
            telemetry.addLine("What auto was ran?\nCROSS - CLOSE\nTRIANGLE - FAR");
            telemetry.update();
            if (gamepad1.a){
                EndingPosition = true;
                autoType = AutoType.CLOSE_15;
            }
            if (gamepad1.y){
                EndingPosition = true;
                autoType = AutoType.ELLIOT_FAR;
            }
        }
        else if (allianceSelect && EndingPosition && rusty == null){
            rusty = new Rusty(this, alliance, autoType);
            rusty.init();
            rusty.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        } else {
            String EndSpot = autoType == AutoType.CLOSE_15 ? "Close" : "Far";
            telemetry.addData("Alliance selected - ", alliance);
            telemetry.addData("Ending Position - ", EndSpot);

            String COPPER = "\u001B[38;2;184;115;51m";
            String RESET = "\u001B[0m";
            String copperhead =
                    "            __ \n" +
                            "           /o \\_ \n" +
                            "           \\__  -<< \n" +
                            "              \\  \\ \n" +
                            "               |  | \n" +
                            "             __/  /___ \n" +
                            "          _-~  x    x ~-_ \n" +
                            "        _-~ x        x   ~-_ \n" +
                            "       / x    _--_     x    \\ \n" +
                            "      | x   _-~   ~-_   x    | \n" +
                            "     @|  x |         | x     | \n" +
                            "       \\ x  \\       / x     / \n" +
                            "        ~-_ x~-----~x   _-~ \n" +
                            "           ~-_______-~ \n";

            telemetry.addLine("\n");
            telemetry.addLine(COPPER + copperhead + RESET);
            telemetry.update();
        }
    }

    @Override
    public void run(){
        rusty.run();
    }
}

package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Globals.States.Alliance;
import org.firstinspires.ftc.teamcode.Globals.Paradigms.Rusty;

@Config
@TeleOp (name = "MainTeleOp", group = "a teleop")
public class MainTeleOp extends CommandOpMode {
    Rusty rusty;
    private boolean allianceSelect = false;
    Alliance alliance;

    @Override
    public void initialize(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad1.setLedColor(0.721568627, .3, 0.1, Gamepad.LED_DURATION_CONTINUOUS);
    }

    @Override
    public void initialize_loop(){
        if (!allianceSelect){
            telemetry.addLine("What alliance?\nCROSS - BLUE\nTRIANGLE - RED");

            if (gamepad1.a){
                alliance = Alliance.BLUE;
                allianceSelect = true;
                rusty = new Rusty(this, alliance);
                rusty.init();
                rusty.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
            }
            if (gamepad1.y){
                alliance = Alliance.RED;
                allianceSelect = true;
                rusty = new Rusty(this, alliance);
                rusty.init();
                rusty.setBulkReading(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
            }
        }
        else {
            telemetry.addData("Alliance selected: ", alliance);
        }
        telemetry.update();
    }

    @Override
    public void run(){
        rusty.run();
    }
}
package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;
import org.firstinspires.ftc.teamcode.Utilities.Constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.Utilities.Storage;

@TeleOp(name="RedCompetitionTeleOP")
public class RedCompetitionTeleOP extends CommandOpMode
{

    private Maelstrom Robot;

    @IgnoreConfigurable
    static TelemetryManager telemetryManager;
    static TelemetryPacket telemetryPacket;
    static FtcDashboard dashboard;

    @Override
    public void initialize()
    {
        telemetryManager= PanelsTelemetry.INSTANCE.getTelemetry();
        dashboard=FtcDashboard.getInstance();
        telemetryPacket= new TelemetryPacket();

        Robot= new Maelstrom(hardwareMap,telemetry, Maelstrom.Alliance.RED,gamepad1,gamepad2);
        telemetry.addData("tempOffset: ", Storage.turretOffset);
        Robot.turret.setManualAngle(0);
        Robot.initializeTeleOP();
        Robot.turret.updateOffset();
        Robot.turret.startPoseTracking();
    }

    @Override
    public void run()
    {
        super.run();

        Robot.shooter.velocityController.setPIDF(ShooterConstants.kP,ShooterConstants.kI,ShooterConstants.kD,ShooterConstants.kF);
        telemetryManager.addData("Velocity: ", Robot.shooter.currentVelocity);
        telemetryManager.addData("Target: ", Robot.shooter.targetVelocity);
        telemetryManager.addLine("Current:" + Robot.shooter.currentVelocity + " Target:" + Robot.shooter.targetVelocity
        );

        telemetryPacket.put("Velocity: ", Robot.shooter.currentVelocity);
        telemetryPacket.put("Target: ", Robot.shooter.targetVelocity);
        dashboard.sendTelemetryPacket(telemetryPacket);

        //telemetryManager.update(telemetry);
        telemetry.update();
    }
}

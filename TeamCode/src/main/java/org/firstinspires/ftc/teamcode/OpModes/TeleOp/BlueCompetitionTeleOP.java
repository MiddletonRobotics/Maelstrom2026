package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.Storage;

import java.util.Optional;

@TeleOp(name="BlueCompetitionTeleOP")
public class BlueCompetitionTeleOP extends CommandOpMode
{
    private Maelstrom Robot;


    @Override
    public void initialize()
    {
        Robot= new Maelstrom(hardwareMap,telemetry, Maelstrom.Alliance.BLUE,gamepad1,gamepad2);
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
        telemetry.update();

        Optional<Pose> visionData = Optional.of(Robot.cams.getPedro());

        if (visionData.isPresent() && Robot.cams.shouldTrustVision(visionData.get(), Robot.dt.getPose())) {
            Pose visionPose = visionData.get();
            Pose visionPose2d = new Pose(
                    visionPose.getX(),
                    visionPose.getY(),
                    visionPose.getHeading()
            );

            telemetry.addData(" Estimated Robot Pose X", visionPose2d.getX());
            telemetry.addData(" Estimated Robot Pose Y", visionPose2d.getY());
            telemetry.addData(" Estimated Robot Pose θ", visionPose2d.getHeading());

            //Robot.dt.updateWithVision(visionPose2d);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Subsystems.Maelstrom.blueCorner1;
import static org.firstinspires.ftc.teamcode.Subsystems.Maelstrom.blueCorner2;
import static org.firstinspires.ftc.teamcode.Subsystems.Maelstrom.blueCorner3;
import static org.firstinspires.ftc.teamcode.Subsystems.Maelstrom.blueCorner4;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.AutoAimTurret;
import org.firstinspires.ftc.teamcode.Commands.DrivetrainController;
import org.firstinspires.ftc.teamcode.Commands.FarShootCommand;
import org.firstinspires.ftc.teamcode.Commands.ParkCommand;
import org.firstinspires.ftc.teamcode.Commands.SOTMFarShootCommand;
import org.firstinspires.ftc.teamcode.Commands.SOTMShootCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCommandV2;
import org.firstinspires.ftc.teamcode.Commands.ShooterDistanceUpdater;
import org.firstinspires.ftc.teamcode.Commands.TeleOPFarShootCommand;
import org.firstinspires.ftc.teamcode.Commands.TeleOPShootCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Maelstrom;

import kotlin.time.Instant;

public class TeleOPBindings {
    private GamepadEx driver1;
    private GamepadEx driver2;

    // Driver 1
    public Button headingReset;
    public Button poseReset;
    public Button closeShoot;
    public Trigger farShoot;
    public Button parkUp;
    public Button parkDown;
    public Button park1;
    public Button park2;
    public Button park3;
    public Button park4;

    // Driver 2
    public Trigger intakeIn;
    public Trigger intakeOut;
    public Button kickerUp;
    public Button gateUp;
    public Button flyWheelOn;
    public Button flyWheelOff;
    public Button closeVelocity;
    public Button midVelocity;
    public Button farVelocity;

    private Maelstrom robot;

    public TeleOPBindings(GamepadEx driver1, GamepadEx driver2, Maelstrom robot) {
        this.robot = robot;
        this.driver1 = driver1;
        this.driver2 = driver2;

        headingReset = new GamepadButton(driver1, GamepadKeys.Button.TOUCHPAD);
        poseReset = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        closeShoot = new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER);
        farShoot = new Trigger(() -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        parkUp = new GamepadButton(driver1, GamepadKeys.Button.A);
        parkDown = new GamepadButton(driver1, GamepadKeys.Button.B);
        park1 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP);
        park2 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT);
        park3 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN);
        park4 = new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT);

        intakeIn = new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        intakeOut = new Trigger(() -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        kickerUp = new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT);
        gateUp = new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP);
        flyWheelOn = new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER);
        flyWheelOff = new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER);
        closeVelocity = new GamepadButton(driver2, GamepadKeys.Button.B);
        midVelocity = new GamepadButton(driver2, GamepadKeys.Button.X);
        farVelocity = new GamepadButton(driver2, GamepadKeys.Button.Y);
    }

    public void controlMap() {
        // Driver1
        headingReset.whenPressed(new InstantCommand(() -> robot.dt.resetHeading(robot.color)));
        poseReset.whenPressed(new InstantCommand(() -> robot.dt.resetPose(robot.color)));

        closeShoot.whenPressed(new SOTMShootCommand(robot));
        farShoot.whenActive(new SOTMFarShootCommand(robot));
        parkUp.whenPressed(new InstantCommand(() -> robot.dt.parkUp()))
                .whenReleased(new InstantCommand(() -> robot.dt.stopPark()));
        parkDown.whenPressed(new InstantCommand(() -> robot.dt.parkDown()))
                .whenReleased(new InstantCommand(() -> robot.dt.stopPark()));

        park1.whenPressed(new ParkCommand(robot, robot.dt::getPose, blueCorner1), true);
        park2.whenPressed(new ParkCommand(robot, robot.dt::getPose, blueCorner2), true);
        park3.whenPressed(new ParkCommand(robot, robot.dt::getPose, blueCorner3), true);
        park4.whenPressed(new ParkCommand(robot, robot.dt::getPose, blueCorner4), true);

        // Driver 2
        intakeIn.whenActive(new InstantCommand(robot.intake::spinIn))
                .whenInactive(new InstantCommand(robot.intake::stop));
        intakeOut.whenActive(new InstantCommand(robot.intake::spinOut))
                .whenInactive(new InstantCommand(robot.intake::stop));
        kickerUp.whenPressed(new InstantCommand(robot.intake::kicker2Up))
                .whenReleased(new InstantCommand(robot.intake::kicker2down));
        gateUp.whenPressed(new InstantCommand(robot.intake::kickerUp))
                .whenReleased(new InstantCommand(robot.intake::kickerDown));
        flyWheelOn.whenPressed(new InstantCommand(robot.shooter::enableFlywheel));
        flyWheelOff.whenPressed(new InstantCommand(robot.shooter::disableFlywheel));
        closeVelocity.whenPressed(new InstantCommand(robot.shooter::shootClose));
        midVelocity.whenPressed(new InstantCommand(robot.shooter::shootMid));
        farVelocity.whenPressed(new InstantCommand(robot.shooter::shootFar));
    }

    public void configureDefaultCommands() {
        robot.dt.setDefaultCommand(
                new DrivetrainController(robot, driver1::getLeftX, driver1::getLeftY, driver1::getRightX));
        robot.turret.setDefaultCommand(new AutoAimTurret(robot, robot.dt::getPose));
        robot.shooter.setDefaultCommand(new ShooterDistanceUpdater(robot));
    }

}

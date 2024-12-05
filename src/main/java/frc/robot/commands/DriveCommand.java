package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Utils.tools;
import frc.robot.Utils.CommandController.ControllerInterface;
import frc.robot.subsystems.Drivebase.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {

    private final Drivebase drive;
    private final ControllerInterface controller;

    ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    Optional<Alliance> ally;
    int isRed;

    public DriveCommand(Drivebase drive, ControllerInterface controller) {
        addRequirements(drive);
        this.drive = drive;
        this.controller = controller;
    }

    @Override
    public void initialize() {
        ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                isRed = 1;
            } else if (ally.get() == Alliance.Blue) {
                isRed = -1;
            }
        }
    }

    @Override
    public void execute() {

        double x = getX();
        double y = getY();

        double angle = Math.atan2(y, x);
        double speed = tools.normalizedPolinominal(Math.hypot(x, y), OperatorConstants.pow, OperatorConstants.n)
                * DrivebaseConstants.maxRobotVelocity;

        targetSpeeds.vxMetersPerSecond = Math.cos(angle) * speed;
        targetSpeeds.vyMetersPerSecond = Math.sin(angle) * speed;
        targetSpeeds.omegaRadiansPerSecond = getOmega();

        Logger.recordOutput("Controller/Driver Input", targetSpeeds);
        drive.setGlobalDrive(targetSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private double getX() {
        return tools.minThreshold(controller.getLeftY(), 0.05) * isRed;
    }

    private double getY() {
        return tools.minThreshold(controller.getLeftX(), 0.05) * isRed;
    }

    public double getOmega() {
        return tools.minThreshold(controller.getRightX(), 0.05) * Constants.DrivebaseConstants.maxRobotOmega
                * Constants.OperatorConstants.invertOmega;
    }
}

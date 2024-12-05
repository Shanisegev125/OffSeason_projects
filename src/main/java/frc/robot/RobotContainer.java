// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.DrivebaseConstants.SwerveModuleConstants;
import frc.robot.Utils.tools;
import frc.robot.Utils.CommandController.ControllerInterface;
import frc.robot.Utils.CommandController.XboxController;
import frc.robot.commands.SubwooferCenteringCommand;
import frc.robot.Utils.CommandController.PS5Controller;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Climb.ClimbIO;
import frc.robot.subsystems.Climb.ClimbIOSim;
import frc.robot.subsystems.Climb.ClimbSubsystem;
import frc.robot.subsystems.Drivebase.Drivebase;
import frc.robot.subsystems.Drivebase.Phoenix6Odomtery;
import frc.robot.subsystems.Drivebase.SwerveModuleIOPhoenix6;
import frc.robot.subsystems.Drivebase.SwerveModuleIOSim;
import frc.robot.subsystems.Feeder.FeederIOPhoenix6;
import frc.robot.subsystems.Feeder.FeederIOSim;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Intake.IntakeIOPhoenix6;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Leds.LEDStrip;
import frc.robot.subsystems.Pivot.PivotIORev;
import frc.robot.subsystems.Pivot.PivotIOSim;
import frc.robot.subsystems.Pivot.PivotSubsytem;
import frc.robot.subsystems.Shooter.ShooterIOPhoenix6;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure.SuperState;
import frc.robot.subsystems.Vision.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

	ControllerInterface driver;
	ControllerInterface operator;

	Drivebase drivebase;
	RobotState robotState;
	SuperStructure superStructure;
	Phoenix6Odomtery phoenix6Odomtery;
	Vision vision;
	IntakeSubsystem intake;
	FeederSubsystem feeder;
	PivotSubsytem pivot;
	ShooterSubsystem shooter;
	ClimbSubsystem climb;
	LEDStrip leds;

	public RobotContainer() {

		driver = switch (OperatorConstants.driverControllerType) {
			case "XBOX" -> new XboxController(OperatorConstants.driverControllerPort);
			case "PS5" -> new PS5Controller(OperatorConstants.driverControllerPort);
			default -> new ControllerInterface() {
			};
		};

		operator = switch (OperatorConstants.driverControllerType) {
			case "XBOX" -> new XboxController(OperatorConstants.operatorControllerPort);
			case "PS5" -> new PS5Controller(OperatorConstants.operatorControllerPort);
			default -> new ControllerInterface() {
			};
		};

		if (Robot.isReal()) {
			drivebase = new Drivebase(
					new SwerveModuleIOPhoenix6(SwerveModuleConstants.FL),
					new SwerveModuleIOPhoenix6(SwerveModuleConstants.FR),
					new SwerveModuleIOPhoenix6(SwerveModuleConstants.BL),
					new SwerveModuleIOPhoenix6(SwerveModuleConstants.BR));

			intake = new IntakeSubsystem(new IntakeIOPhoenix6(Ports.INTAKE_MOTOR, Ports.INTAKE_BEAMBREAKER_DIO));
			feeder = new FeederSubsystem(new FeederIOPhoenix6(Ports.FEEDER_MOTOR, Ports.FEEDER_BEAMBREAKER_DIO));
			pivot = new PivotSubsytem(new PivotIORev(Ports.PIVOT_LEFT_MOTOR, Ports.PIVOT_RIGHT_MOTOR, Ports.PIVOT_LIMITSWITCH_DIO));
			shooter = new ShooterSubsystem(new ShooterIOPhoenix6(
					Ports.SHOOTER_LOWER_MOTOR,
					Ports.SHOOTER_UPPER_MOTOR,
					Ports.SHOOTER_BEAMBREAKER_DIO));
			// climb = new ClimbSubsystem(new ClimbIORev(Ports.CLIMB_MOTOR));
			climb = new ClimbSubsystem(new ClimbIO() {});

		} else {
			drivebase = new Drivebase(
					new SwerveModuleIOSim(SwerveModuleConstants.FL),
					new SwerveModuleIOSim(SwerveModuleConstants.FR),
					new SwerveModuleIOSim(SwerveModuleConstants.BL),
					new SwerveModuleIOSim(SwerveModuleConstants.BR));

			intake = new IntakeSubsystem(new IntakeIOSim());
			feeder = new FeederSubsystem(new FeederIOSim());
			pivot = new PivotSubsytem(new PivotIOSim());
			shooter = new ShooterSubsystem(new ShooterIOSim());
			climb = new ClimbSubsystem(new ClimbIOSim());
		}

		robotState = new RobotState(drivebase);
		vision = new Vision();
		phoenix6Odomtery = new Phoenix6Odomtery(robotState, drivebase);
		leds = new LEDStrip(Ports.LED_STRIP_PWM, LedConstants.STRIP_LENGTH);

		superStructure = new SuperStructure(
				robotState,
				drivebase,
				vision,
				intake,
				feeder,
				pivot,
				shooter,
				leds);

		AutoBuilder.configureHolonomic(
				robotState::getPose,
				robotState::resetPose,
				drivebase::getLocalChassisSpeeds,
				drivebase::setLocalDrive,
				DrivebaseConstants.pathFollowerConfig,
				tools::isRedAlliance,
				drivebase);
		PathfindingCommand.warmupCommand().schedule();

		configureBindings();
	}

	private void configureBindings() {

		// Driver
		driver.down().toggleOnTrue(
				new InstantCommand(
						() -> {
							robotState.resetPose(new Pose2d());
						}));

		driver.leftBumper().toggleOnTrue(
				superStructure.setWantedSuperStateCommand(SuperState.INTAKE)).toggleOnFalse(
						superStructure.setWantedSuperStateCommand(SuperState.IDLE));

		driver.leftTrigger().toggleOnTrue(superStructure.setWantedSuperStateCommand(SuperState.SHOOT_SPEAKER));

		driver.right().toggleOnTrue(superStructure.setWantedSuperStateCommand(SuperState.EJECT))
				.toggleOnFalse(superStructure.setWantedSuperStateCommand(SuperState.IDLE));

		driver.rightBumper().toggleOnTrue(superStructure.setWantedSuperStateCommand(SuperState.EJECT_SHOOTER))
				.toggleOnFalse(superStructure.setWantedSuperStateCommand(SuperState.IDLE));

		driver.pov0().toggleOnTrue(new SubwooferCenteringCommand(drivebase, robotState));

		// Operator
		// operator.leftTrigger().toggleOnTrue(new ClimbCenteringCommand(drivebase, robotState).withTimeout(4));

		operator.leftBumper().toggleOnTrue(intake.setOverloadStateCommand(IntakeSubsystem.SystemState.INTAKE))
				.toggleOnFalse(intake.setOverloadStateCommand(null));

		operator.right().toggleOnTrue(intake.setOverloadStateCommand(IntakeSubsystem.SystemState.EJECT))
				.toggleOnFalse(intake.setOverloadStateCommand(null));
		operator.pov0().toggleOnTrue(feeder.setOverloadStateCommand(FeederSubsystem.SystemState.FEED))
				.toggleOnFalse(feeder.setOverloadStateCommand(null));
		operator.pov180()
				.toggleOnTrue(feeder.setOverloadStateCommand(FeederSubsystem.SystemState.EJECT))
				.toggleOnFalse(feeder.setOverloadStateCommand(null));
		operator.rightBumper().toggleOnTrue(shooter.setOverloadStateCommand(ShooterSubsystem.SystemState.SUBWOOFER))
				.toggleOnFalse(shooter.setOverloadStateCommand(null));

	}

	public Drivebase getDrivebase() {
		return drivebase;
	}

	public RobotState getRobotState() {
		return robotState;
	}

	public SuperStructure getSuperStructure() {
		return superStructure;
	}

	public Phoenix6Odomtery getPhoenix6Odomtery() {
		return phoenix6Odomtery;
	}

	public Vision getVision() {
		return vision;
	}

	public IntakeSubsystem getIntake() {
		return intake;
	}

	public FeederSubsystem getFeeder() {
		return feeder;
	}

	public PivotSubsytem getPivot() {
		return pivot;
	}

	public ShooterSubsystem getShooter() {
		return shooter;
	}

	public ClimbSubsystem getClimb() {
		return climb;
	}

	public LEDStrip getLEDs() {
		return leds;
	}
}

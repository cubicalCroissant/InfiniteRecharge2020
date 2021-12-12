/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.auto.FollowPath;
import frc.robot.commands.climber.LowerTelescopingArm;
import frc.robot.commands.climber.RaiseTelescopingArm;
import frc.robot.commands.climber.WinchUp;
import frc.robot.commands.drivetrain.DriveWithXboxController;
import frc.robot.commands.intake.DriveIntake;
import frc.robot.commands.shooter.DriveAgitator;
import frc.robot.commands.shooter.DriveShooter;
import frc.robot.commands.shooter.WaitForSpeed;
import frc.robot.commands.spinner.DriveSpinner;
import frc.robot.commands.spinner.ResetSpinnerPosition;
import frc.robot.commands.spinner.SpinnerDown;
import frc.robot.commands.spinner.SpinnerUp;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spinner;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private XboxController m_driverController = new XboxController(0);
    private XboxController m_operatorController = new XboxController(1);
    private AHRS m_ahrs = new AHRS(Port.kMXP);
    private DriveTrain m_driveTrain = new DriveTrain(m_ahrs);
    private Climber m_climber = new Climber();
    private Shooter m_shooter = new Shooter();
    private Intake m_intake = new Intake();
    private Spinner m_spinner = new Spinner();

    private SendableChooser<Command> m_commandChooser = new SendableChooser<Command>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        CameraServer.getInstance().startAutomaticCapture(0);
        SmartDashboard.putData("Shooter", m_shooter);
        m_driveTrain.setDefaultCommand(new DriveWithXboxController(m_driveTrain, m_driverController));
        configureButtonBindings();
        m_commandChooser.setDefaultOption("Drive Forward", new ParallelCommandGroup(
                new FollowPath(m_driveTrain, Trajectories.simpleForward),
                new ResetSpinnerPosition(m_spinner)));
        m_commandChooser.addOption("Reset Spinner Only", new ResetSpinnerPosition(m_spinner));
        m_commandChooser.addOption("Drive And Shoot", new ParallelCommandGroup(
                new ResetSpinnerPosition(m_spinner),
                new AutoShoot(m_shooter, m_intake, m_driveTrain, Trajectories.shootDriveForward)
            )
        );
        m_commandChooser.addOption("Drive Circle", new FollowPath(m_driveTrain, Trajectories.circleRight));
        m_commandChooser.addOption("Backwards P", new FollowPath(m_driveTrain, Trajectories.backwardsP));
        
        SmartDashboard.putData("Auto Command", m_commandChooser);
        SmartDashboard.putNumber("Auto Delay", 0);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        JoystickButton shiftButton = new JoystickButton(m_driverController, XboxController.Button.kBumperLeft.value);
        JoystickButton operatorShiftButton = new JoystickButton(m_operatorController, XboxController.Button.kBumperLeft.value);
        
        // Drivetrain
        new POVButton(m_driverController, 0).whenActive(()-> m_driveTrain.setThrottle(0.9));
        new POVButton(m_driverController, 270).whenActive(()-> m_driveTrain.setThrottle(0.6));
        new POVButton(m_driverController, 180).whenActive(()-> m_driveTrain.setThrottle(0.3));
        
        JoystickButton backUpButton = new JoystickButton(m_driverController, XboxController.Button.kStart.value);
        backUpButton.whenPressed(new FollowPath(m_driveTrain, Trajectories.nineInchesBack));

        // Climber
        JoystickButton winchUpButton = new JoystickButton(m_operatorController, XboxController.Button.kBack.value);
        JoystickButton telescopingArmButton = new JoystickButton(m_operatorController, XboxController.Button.kStart.value);

        telescopingArmButton.and(operatorShiftButton.negate()).whileActiveContinuous(new RaiseTelescopingArm(m_climber));
        telescopingArmButton.and(operatorShiftButton).whileActiveContinuous(new LowerTelescopingArm(m_climber));

        winchUpButton.whileHeld(new WinchUp(m_climber));

        // Intake
        JoystickButton driveIntakeButton = new JoystickButton(m_driverController, XboxController.Button.kBumperRight.value);
        JoystickButton operatorDriveIntakeButton = new JoystickButton(m_operatorController, XboxController.Button.kBumperRight.value);

        driveIntakeButton.and(shiftButton.negate()).whileActiveContinuous(new DriveIntake(m_intake, false));
        driveIntakeButton.and(shiftButton).whileActiveContinuous(new DriveIntake(m_intake, true));
        operatorDriveIntakeButton.and(operatorShiftButton.negate()).whileActiveContinuous(new DriveIntake(m_intake, false));
        operatorDriveIntakeButton.and(operatorShiftButton).whileActiveContinuous(new DriveIntake(m_intake, true));

        // Shooter
        JoystickButton driveShooterButton = new JoystickButton(m_operatorController, XboxController.Button.kA.value);
        JoystickButton feedShooterButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);

        SequentialCommandGroup feedShooterCommand = new SequentialCommandGroup(
                                                        new WaitForSpeed(m_shooter),
                                                        new ParallelCommandGroup(
                                                            new DriveAgitator(m_shooter),
                                                            new DriveIntake(m_intake, false)
                                                        )
                                                    );

        driveShooterButton.toggleWhenPressed(new DriveShooter(m_shooter));
        feedShooterButton.whileHeld(feedShooterCommand);
        
        // Spinner
        JoystickButton driveSpinnerButton = new JoystickButton(m_operatorController, XboxController.Button.kX.value);
        JoystickButton spinnerUp = new JoystickButton(m_driverController, XboxController.Button.kX.value);

        driveSpinnerButton.and(operatorShiftButton.negate()).whileActiveContinuous(new DriveSpinner(m_spinner, false));
        driveSpinnerButton.and(operatorShiftButton).whileActiveContinuous(new DriveSpinner(m_spinner, true));

        spinnerUp.and(shiftButton.negate()).whenActive(new SpinnerUp(m_spinner));
        spinnerUp.and(shiftButton).whenActive(new SpinnerDown(m_spinner));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new WaitUntilCommand(SmartDashboard.getNumber("Auto Delay", 0)),
            m_commandChooser.getSelected()
        );
    }
}
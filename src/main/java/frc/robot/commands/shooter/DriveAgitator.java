/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class DriveAgitator extends CommandBase {
    private Shooter m_shooter;

    /**
     * Creates a new FeedShooter.
     */
    public DriveAgitator(Shooter shooter) {
        m_shooter = shooter;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double agitatorSpeed = SmartDashboard.getNumber("Speed Agitator", 0.5);
        double feederSpeed = SmartDashboard.getNumber("Speed Feeder", 0.5);
        m_shooter.driveFeeder(feederSpeed);
        m_shooter.driveAgitator(agitatorSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.driveAgitator(0);
        m_shooter.driveFeeder(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
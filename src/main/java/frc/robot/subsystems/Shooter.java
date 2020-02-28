/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.Shooter.*;

public class Shooter extends SubsystemBase {
    private CANSparkMax m_shooterMain = new CANSparkMax(CAN.kShooterMain, MotorType.kBrushless);
    private CANSparkMax m_shooterFeeder = new CANSparkMax(CAN.kShooterFeeder, MotorType.kBrushless);
    private CANSparkMax m_shooterAgitator = new CANSparkMax(CAN.kShooterAgitator, MotorType.kBrushless);

    /**
     * Creates a new Shooter.
     */
    public Shooter() {
        SmartDashboard.putNumber("Speed Main", 2925);
        SmartDashboard.putNumber("Speed Feeder", 0.5);
        SmartDashboard.putNumber("Speed Agitator", 0.5);

        m_shooterMain.restoreFactoryDefaults();
        m_shooterMain.getEncoder().setPosition(0);
        m_shooterMain.setInverted(true);
        m_shooterMain.setClosedLoopRampRate(2);

        m_shooterFeeder.restoreFactoryDefaults();
        m_shooterFeeder.getEncoder().setPosition(0);
        m_shooterFeeder.setInverted(true);

        m_shooterAgitator.restoreFactoryDefaults();
        m_shooterAgitator.getEncoder().setPosition(0);
    }

    public boolean isUpToSpeed() {
        return m_shooterMain.getEncoder().getVelocity() > kSpeedThreshold;
    }

    public void driveMain(double speed) {
        if (m_shooterMain.getEncoder().getVelocity() >= speed) {
            m_shooterMain.getPIDController().setP(0);
        } else {
            m_shooterMain.getPIDController().setP(kP);
        }

        m_shooterMain.getPIDController().setFF(kF);
        m_shooterMain.getPIDController().setReference(speed, ControlType.kVelocity);
    }

    public void driveFeeder(double speed) {
        m_shooterFeeder.set(speed);
    }

    public void driveAgitator(double speed) {
        m_shooterAgitator.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Shooter", this);
        SmartDashboard.putNumber("Shooter Velocity", m_shooterMain.getEncoder().getVelocity());
    }
}
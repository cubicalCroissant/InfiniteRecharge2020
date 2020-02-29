/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Spinner.*;
import static frc.robot.Constants.Ports.*;

public class Spinner extends SubsystemBase {
    private CANSparkMax m_spinnerLeftRight = new CANSparkMax(CAN.kSpinnerLeftRight, MotorType.kBrushless);
    private CANSparkMax m_spinnerUpDown = new CANSparkMax(CAN.kSpinnerUpDown, MotorType.kBrushless);

    private CANEncoder m_spinnerLeftRightEncoder = m_spinnerLeftRight.getEncoder();
    private CANEncoder m_spinnerUpDownEncoder = m_spinnerUpDown.getEncoder();

    private CANPIDController m_spinnerUpDownPIDController = m_spinnerUpDown.getPIDController();

    private ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);
    private ColorMatch m_colorMatcher = new ColorMatch();

    /**
     * Creates a new Spinner.
     */
    public Spinner() {
        SmartDashboard.putNumber("Spinner Speed", 0.5);
        SmartDashboard.putData(this);

        m_spinnerLeftRight.restoreFactoryDefaults();
        m_spinnerLeftRightEncoder.setPosition(0);
        m_spinnerLeftRight.setIdleMode(IdleMode.kBrake);

        m_spinnerUpDown.restoreFactoryDefaults();
        m_spinnerUpDownEncoder.setPosition(0);
        m_spinnerUpDown.setIdleMode(IdleMode.kBrake);

        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
    }

    public Color readColor() {
        Color color = m_colorSensor.getColor();
        ColorMatchResult result = m_colorMatcher.matchClosestColor(color);
        return result.color;
    }

    public void spin(double speed) {
        m_spinnerLeftRight.set(speed);
    }

    public boolean isAtLowerLimit() {
        return m_spinnerUpDown.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).get();
    }

    public void initialDriveDown() {
        m_spinnerUpDown.set(0.1);
    }

    public void resetPosition() {
        m_spinnerUpDownEncoder.setPosition(0);
    }

    public void raise() {

        while (m_spinnerUpDownEncoder.getPosition() > -12) {
            m_spinnerUpDown.set(-0.2);
        }
        m_spinnerUpDown.set(0);
        SmartDashboard.putBoolean("Spinner Stowed", false);
    }

    public void lower() {
        while (m_spinnerUpDownEncoder.getPosition() < 0) {
            m_spinnerUpDown.set(0.2);
        }
        m_spinnerUpDown.set(0);
        SmartDashboard.putBoolean("Spinner Stowed", true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spinner Up Down Encoder", m_spinnerUpDownEncoder.getPosition());
    }
}

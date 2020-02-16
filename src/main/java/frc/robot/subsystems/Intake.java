package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ScaledJoystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.*;

public class Intake extends SubsystemBase {
    private CANSparkMax m_intakeSparkMax = new CANSparkMax(kIntake, MotorType.kBrushless);
    private ScaledJoystick m_joystick = new ScaledJoystick(1);
    private JoystickButton m_intakeButton = new JoystickButton(m_joystick, 4);
    
    public Intake() {
        SmartDashboard.putNumber("Speed", 0.5);
        SmartDashboard.putBoolean("Reversed", false);
        m_intakeSparkMax.getEncoder().setPositionConversionFactor(0);
        m_intakeSparkMax.getEncoder().setPosition(0);
    }
    
    @Override
    public void periodic() {
        double speed = SmartDashboard.getNumber("Speed", 0.5);
        boolean reversed = SmartDashboard.getBoolean("Reversed", false);
        m_intakeSparkMax.setInverted(reversed);
        if (m_intakeButton.get()){
            m_intakeSparkMax.set(speed);
        }
    }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Intake.IntakeMode;
import frc.robot.commands.intake.DriveIntake;
import frc.robot.commands.shooter.DriveAgitator;
import frc.robot.commands.shooter.DriveShooter;
import frc.robot.commands.shooter.WaitForSpeed;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
    /**
     * Creates a new AutoShoot.
     */
    public AutoShoot(Shooter shooter, Intake intake, DriveTrain driveTrain, Trajectory traj) {
        super(
            new ParallelCommandGroup(
                new DriveShooter(shooter), 
                new FollowPath(driveTrain, traj)
            ),

            new WaitForSpeed(shooter), 
            
            new ParallelCommandGroup(
                new DriveAgitator(shooter), 
                new DriveIntake(intake, false, IntakeMode.Inner)
            ),

            new WaitCommand(10),

            new InstantCommand(() -> {
                shooter.driveAgitator(0);
                shooter.driveFeeder(0);
                shooter.driveMain(0);

                intake.driveMotors(0, 0);
                
                driveTrain.driveVolts(0, 0);
            })
        );
    }
}

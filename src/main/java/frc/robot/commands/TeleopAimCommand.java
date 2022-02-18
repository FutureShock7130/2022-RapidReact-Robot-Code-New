package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TeleopAimCommand extends CommandBase {
    // This command is not going to be used in the competition
    // Sorry Aaron!!

    // This command is used to allow the robot to aim at a specific coordinate at any point on the field.
    // Future update will make its target adjustable without changing the class itself. 

    DriveTrainSubsystem m_robotDrive;
    Translation2d initTranslation;
    Rotation2d initRotation;
    Rotation2d changeAngle; 
    Rotation2d finalRotation;
    Rotation2d currentRotation;
    double targetX = 3;
    double targetY = 4;
    
    public TeleopAimCommand(DriveTrainSubsystem robotDrive){
            m_robotDrive = robotDrive;
            addRequirements(robotDrive);
    }
    
    @Override
    public void initialize(){
            initTranslation = m_robotDrive.getPose().getTranslation();
            initRotation = m_robotDrive.getPose().getRotation();
            // get translation2d object and rotation2d object from the Pose2d object from the DriveSubsysten.
            double xDiff = targetX - initTranslation.getX();
            // Calculate difference in x coordinate between the target coordinate and the current coordinate
            double yDiff = targetY - initTranslation.getY();
            // Calculate difference in y coordinate between the target coordinate and the current coordinate
            double changeCalc = Math.atan(yDiff/xDiff);
            // Calculate the angle difference between the two coordinate (if the current rotation(angle) is 0)
            changeAngle = new Rotation2d(changeCalc).plus(initRotation.times(-1));
            // Add the current angle into the rotation. Think of it as first correcting the angle of the robot to 0, 
            // then turn the actual angle difference between the two coordinates.
            changeAngle = changeAngle.getRadians()>Math.PI ? changeAngle.minus(new Rotation2d(2*Math.PI)) : changeAngle;
            // if the angle is greater than Pi, the angle is too big (larger than half the circle) 
            // and it's better to rotate the other way around. So I used this inline if statement (?: statement) to assign the angle
            // if the angle is greater than Pi, than it should turn the angle-2pi (draw a diagram to help you think about it)
            finalRotation = initRotation.rotateBy(changeAngle);
            // Calculate the final angle that the robot should be at, which will be used to determine when the rotation should stop
    }

    @Override
    public void execute(){
            m_robotDrive.drive(0,0, changeAngle.getRadians()/2, true);
            // divided by 2 to move slower but more precise
            currentRotation = m_robotDrive.getPose().getRotation();
            // update rotation for comparison
    }

    public boolean isFinished(){
            while (Math.abs(finalRotation.minus(currentRotation).getRadians())>0.05){
                    // if the current rotation is not 0.05 radians within the target angle, keep rotating
                    return false;
            }
            return true;
            // stops the function by returning true
    }
}
package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.SwerveDriveStopCommand;
import frc.robot.subsystems.drive.DriveBase;
import stl.sysId.CharacterizableSubsystem;

public class AutoFactory {
    private final Supplier<List<Object>> responses;
    private final DriveBase driveBase;

    public AutoFactory(DriveBase driveBase,
            Supplier<List<Object>> responsesSupplier) {
        this.responses = responsesSupplier;
        this.driveBase = driveBase;

        AutoBuilder.configureHolonomic(driveBase::getPose, // Robot pose supplier
                driveBase::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                driveBase::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                driveBase::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                DriveConstants.PATH_FOLLOWER_CONFIG, () -> {
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                }, driveBase);
    }

    /**
     * Determines type of an inputted trajectory - constructed using Choreo, or
     * using PathPlanner. Default should be Choreo.
     */
    public enum PathType {
        CHOREO, PATHPLANNER
    }

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose The desired end pose of the generated path.
     * @return The constructed path following command
     */
    public Command getPathFindToPoseCommand(Pose2d targetPose) {

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(targetPose, PathConstants.CONSTRAINTS, 0.0, // Goal
                                                                                                                   // end
                                                                                                                   // velocity
                                                                                                                   // in
                                                                                                                   // meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        ).andThen(new SwerveDriveStopCommand(driveBase));

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose Supplier for the desired end pose of the generated path.
     * @return The constructed path following command
     */
    public Command getPathFindToPoseCommand(Supplier<Pose2d> targetPose) {

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(targetPose.get(), PathConstants.CONSTRAINTS, 0.0, // Goal
                                                                                                                         // end
                                                                                                                         // velocity
                                                                                                                         // in
                                                                                                                         // meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        ).andThen(new SwerveDriveStopCommand(driveBase));

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command to a preset path from the deploy
     * directory Path can be PathPlanner or Choreo-constructed.
     * 
     * @param pathname A String containing the name of the file with the path (leave
     *                 out the .traj or .path ending).
     * @param pathType A {@link PathType} determining the format of the inputted
     *                 trajectory. Files ending in .path should be imported as
     *                 PATHPLANNER, while files ending in .traj should be imported
     *                 as CHOREO.
     * @return The constructed path following command
     */
    public Command getPathFindToPathCommand(String pathname, PathType pathType) {
        PathPlannerPath path;
        switch (pathType) {
            case CHOREO:
                path = PathPlannerPath.fromChoreoTrajectory(pathname);
                break;
            case PATHPLANNER:
                path = PathPlannerPath.fromPathFile(pathname);
                break;
            default:
                path = PathPlannerPath.fromChoreoTrajectory(pathname);
        }

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, PathConstants.CONSTRAINTS, 0.0 // Rotation
                                                                                                             // delay
                                                                                                             // distance
                                                                                                             // in
                                                                                                             // meters.
                                                                                                             // This is
                                                                                                             // how far
                                                                                                             // the
                                                                                                             // robot
                                                                                                             // should
                                                                                                             // travel
                                                                                                             // before
                                                                                                             // attempting
                                                                                                             // to
                                                                                                             // rotate.
        );

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command through a provided set of waypoints. Ends
     * with desired holonomic rotation.
     * 
     * @param goalEndRotationHolonomic Desired holonomic end rotation
     * @param poses                    List of bezier poses. Each {@link Pose2d}
     *                                 represents one waypoint. The rotation
     *                                 component of the pose should be the direction
     *                                 of travel. Do not use holonomic rotation.
     * @return The constructed path following command through provided poses, with
     *         set end rotation.
     */
    public Supplier<Command> getPathFromWaypoints(Rotation2d goalEndRotationHolonomic, Pose2d... poses) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(bezierPoints, PathConstants.CONSTRAINTS,
                new GoalEndState(0.0, goalEndRotationHolonomic) // Goal end state. You can set a holonomic rotation
                                                                // here. If using a differential drivetrain, the
                                                                // rotation will have no effect.
        );

        Supplier<Command> pathCommand = () -> AutoBuilder.pathfindThenFollowPath(path, PathConstants.CONSTRAINTS, 0.0 // Rotation
                                                                                                                      // delay
                                                                                                                      // distance
                                                                                                                      // in
                                                                                                                      // meters.
                                                                                                                      // This
                                                                                                                      // is
                                                                                                                      // how
                                                                                                                      // far
                                                                                                                      // the
                                                                                                                      // robot
                                                                                                                      // should
                                                                                                                      // travel
                                                                                                                      // before
                                                                                                                      // attempting
                                                                                                                      // to
                                                                                                                      // rotate.
        );
        return pathCommand;
    }

    public enum CharacterizationRoutine {
        QUASISTATIC_FORWARD, QUASISTATIC_BACKWARD, DYNAMIC_FORWARD, DYNAMIC_BACKWARD,
    }

    public Command getCharacterizationRoutine() {
        CharacterizableSubsystem subsystem = (CharacterizableSubsystem) responses.get().get(0);
        CharacterizationRoutine routine = (CharacterizationRoutine) responses.get().get(1);

        var sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism((Measure<Voltage> voltage) -> subsystem.runVolts(voltage.in(Volts)), null, // No
                                                                                                                      // log
                                                                                                                      // consumer,
                                                                                                                      // since
                                                                                                                      // data
                                                                                                                      // is
                                                                                                                      // recorded
                                                                                                                      // by
                                                                                                                      // AdvantageKit
                        subsystem));
        switch (routine) {
            case QUASISTATIC_FORWARD:
                return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
            case QUASISTATIC_BACKWARD:
                return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
            case DYNAMIC_FORWARD:
                return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
            case DYNAMIC_BACKWARD:
                return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
            default:
                return new WaitCommand(1);
        }
    }
}

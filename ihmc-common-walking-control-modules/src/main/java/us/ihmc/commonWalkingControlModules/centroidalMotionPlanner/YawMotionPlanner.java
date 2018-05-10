package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/** 
 * Generates a simplified plan to enable a biped to twist while jumping.
 * The body of the humanoid is divided into two parts, upper and lower body. The lower body is stationary w.r.t.
 * {@code ReferenceFrame#getWorldFrame()} during launch and landing states. The upper body is used to generate 
 * angular momentum in the launch and landing phases to make the whole robot twist
 * @author Apoorv S
 *
 */
public class YawMotionPlanner
{
   private final static int numberOfTrajectoryCoefficients = 7;

   private final YoDouble launchTime;
   private final YoDouble flightTime;
   private final YoDouble landingTime;
   private final YoDouble initialUpperBodyYaw;
   private final YoDouble finalUpperBodyYaw;
   private final YoDouble initialLowerBodyYaw;
   private final YoDouble finalLowerBodyYaw;
   private final Trajectory upperBodyLaunchRotation;
   private final Trajectory upperBodyFlightRotation;
   private final Trajectory upperBodyLandRotation;
   private final Trajectory upperBodyFlightAcceleration;
   private final Trajectory lowerBodyFlightRotation;
   private final Trajectory lowerBodyTorque;
   private final String namePrefix;
   private final YoDouble lowerBodyInertia;
   private final YoDouble upperBodyInertia;

   public YawMotionPlanner(YoVariableRegistry registry)
   {
      this.namePrefix = getClass().getSimpleName();

      this.launchTime = new YoDouble(namePrefix + "LaunchDuration", registry);
      this.flightTime = new YoDouble(namePrefix + "FlightDuration", registry);
      this.landingTime = new YoDouble(namePrefix + "LandingDuration", registry);

      this.initialUpperBodyYaw = new YoDouble(namePrefix + "InitialUpperBodyYaw", registry);
      this.finalUpperBodyYaw = new YoDouble(namePrefix + "FinalUpperBodyYaw", registry);
      this.initialLowerBodyYaw = new YoDouble(namePrefix + "InitialLowerBodyYaw", registry);
      this.finalLowerBodyYaw = new YoDouble(namePrefix + "FinalLowerBodyYaw", registry);

      this.upperBodyLaunchRotation = new Trajectory(numberOfTrajectoryCoefficients);
      this.upperBodyFlightRotation = new Trajectory(numberOfTrajectoryCoefficients);
      this.upperBodyLandRotation = new Trajectory(numberOfTrajectoryCoefficients);
      this.upperBodyFlightAcceleration = new Trajectory(numberOfTrajectoryCoefficients);
      this.lowerBodyFlightRotation = new Trajectory(numberOfTrajectoryCoefficients);
      this.lowerBodyTorque = new Trajectory(numberOfTrajectoryCoefficients);
      
      this.lowerBodyInertia = new YoDouble(namePrefix + "LowerBodyInertia", registry);
      this.upperBodyInertia = new YoDouble(namePrefix + "UpperBodyInertia", registry);
   }

   public void compute()
   {
      generateLowerBodyFlightPlan();
      computeUpperBodyConstraintsFromLowerBodyMotion();
      generateUpperBodyPlan();
   }

   private void generateUpperBodyPlan()
   {

   }

   private void computeUpperBodyConstraintsFromLowerBodyMotion()
   {
      TrajectoryMathTools.scale(upperBodyFlightAcceleration, lowerBodyTorque, -1.0 / upperBodyInertia.getDoubleValue());
      
   }

   /**
    * Generates a yaw angle trajectory for the lower body using 
    */
   private void generateLowerBodyFlightPlan()
   {
      double initialAngle = initialLowerBodyYaw.getDoubleValue();
      double finalAngle = finalLowerBodyYaw.getDoubleValue();
      lowerBodyFlightRotation.setQuinticWithZeroTerminalVelocityAndAcceleration(0.0, initialAngle, flightTime.getDoubleValue(), finalAngle);
      lowerBodyFlightRotation.getDerivative(lowerBodyTorque, 2);
      TrajectoryMathTools.scale(lowerBodyTorque, lowerBodyInertia.getDoubleValue());
   }
}

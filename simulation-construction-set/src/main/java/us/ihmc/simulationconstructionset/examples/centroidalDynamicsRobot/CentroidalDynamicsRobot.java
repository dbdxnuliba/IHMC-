package us.ihmc.simulationconstructionset.examples.centroidalDynamicsRobot;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

/**
 * A customizable robot that represents the CoM of a system. Some things that you should 
 * be able to do
 * <ul>
 * <li> add custom feet visualizations  
 * <li> add controllers to control the system
 * <li> 
 * </ul>
 * @author Apoorv S
 *
 */
public class CentroidalDynamicsRobot
{
   private final String robotName;
   private final RobotDescription robotDescription;
   private final CentroidalRobotPhysicalProperties physicalProperties;
   private final Point3D defaultInitialPosition = new Point3D();
   private final Quaternion defaultInitialOrientation = new Quaternion();

   public CentroidalDynamicsRobot(String robotName, CentroidalRobotPhysicalProperties physicalProperties)
   {
      this.robotName = robotName;
      this.robotDescription = new CentroidalRobotDescription(physicalProperties);
      this.physicalProperties = physicalProperties;
      intializeDefaults();
   }

   private void intializeDefaults()
   {
      defaultInitialPosition.set(0.0, 0.0, physicalProperties.getNominalHeight());
      defaultInitialOrientation.setToZero();
   }

   public FloatingRootJointRobot addControllerAndCreateRobot(CentroidalRobotController controller, YoGraphicsListRegistry graphicsListRegistry)
   {
      FloatingRootJointRobot floatingRootJointRobot = new FloatingRootJointRobot(getRobotDescription());
      initialize(floatingRootJointRobot, defaultInitialPosition, defaultInitialOrientation);
      CentroidalRobotEstimator estimator = new CentroidalRobotEstimator(robotName, floatingRootJointRobot.getRootJoint(), graphicsListRegistry);
      CentroidalRobotControllerOutputWriter outputWriter = new CentroidalRobotControllerOutputWriter(robotName, controller, floatingRootJointRobot.getRootJoint(), graphicsListRegistry);
      estimator.setStateToUpdate(outputWriter.getState());
      floatingRootJointRobot.setController(estimator);
      estimator.initialize();
      floatingRootJointRobot.setController(outputWriter);
      estimator.initialize();
      outputWriter.initialize();
      return floatingRootJointRobot;
   }

   private void initialize(FloatingRootJointRobot floatingRootJointRobot, Point3DReadOnly initialPosition, QuaternionReadOnly initialOrientation)
   {
      floatingRootJointRobot.setPositionInWorld(initialPosition);
      floatingRootJointRobot.setOrientation(initialOrientation);
   }

   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }
}
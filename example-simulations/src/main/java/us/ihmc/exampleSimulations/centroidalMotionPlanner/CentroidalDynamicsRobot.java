package us.ihmc.exampleSimulations.centroidalMotionPlanner;

import java.awt.Color;
import java.util.EnumSet;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.avatar.drcRobot.RobotPhysicalProperties;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelFromDescription;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * A customization robot that represents the CoM of a system
 * @author Apoorv S
 *
 */
public class CentroidalDynamicsRobot implements FullRobotModelFactory
{
   private final String robotName;
   private final RobotDescription robotDescription;
   private final JointNameMap<RobotCentroidal> jointMap;
   private final CentroidalRobotPhysicalProperties physicalProperties;

   public CentroidalDynamicsRobot(String robotName, CentroidalRobotPhysicalProperties physicalProperties)
   {
      this.robotName = robotName;
      this.robotDescription = new CentroidalRobotDescription(robotName);
      this.jointMap = new CentroidalRobotJointMap();
      this.physicalProperties = physicalProperties;
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   @Override
   public FullRobotModel createFullRobotModel()
   {
      RobotDescription robotDescription = new CentroidalRobotDescription(robotName);
      FullRobotModelFromDescription fullRobotModel = new FullRobotModelFromDescription(robotDescription, getJointMap(), null);
      return fullRobotModel;
   }

   public JointNameMap<RobotCentroidal> getJointMap()
   {
      return jointMap;
   }

   public enum RobotCentroidal implements RobotSegment<RobotCentroidal>
   {
      BODY;

      public static final RobotCentroidal[] values = values();
      public static final EnumSet<RobotCentroidal> set = EnumSet.allOf(RobotCentroidal.class);

      @Override
      public RobotCentroidal[] getValues()
      {
         return values;
      }

      @Override
      public Class<RobotCentroidal> getClassType()
      {
         return RobotCentroidal.class;
      }

      @Override
      public EnumSet<RobotCentroidal> getEnumSet()
      {
         return set;
      }
   }

   public class CentroidalRobotInitialSetup
   {
      Vector3D initialPosition = new Vector3D();
      Quaternion orientation = new Quaternion();

      public CentroidalRobotInitialSetup()
      {
         
      }

      public void initializeRobot(Robot robot)
      {
         FloatingJoint rootJoint = (FloatingJoint) robot.getRootJoints().get(0);
         rootJoint.setPosition(initialPosition);
         rootJoint.setQuaternion(orientation);
      }

      public void setInitialYaw(double yaw)
      {
         orientation.setToYawQuaternion(yaw);
      }

      public double getInitialYaw()
      {
         return orientation.getYaw();
      }

      public void setInitialGroundHeight(double groundHeight)
      {
         initialPosition.setZ(groundHeight);
      }

      public double getInitialGroundHeight()
      {
         return initialPosition.getZ();
      }

      public void setOffset(Vector3D additionalOffset)
      {
         initialPosition.add(additionalOffset);
      }

      public void getOffset(Vector3D offsetToPack)
      {
         offsetToPack.set(initialPosition);
      }
   }

   public class CentroidalRobotEstimator implements RobotController
   {
      private final FloatingJoint estimatedRootJoint;
      private final FloatingInverseDynamicsJoint controllerRootJoint;
      private final Point3D position = new Point3D();
      private final Vector3D velocity = new Vector3D();
      private final Vector3D acceleration = new Vector3D();
      private final Quaternion orientation = new Quaternion();
      private final Vector3D angularVelocity = new Vector3D();
      private final Vector3D angularAcceleration = new Vector3D();
      private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      private final FullRobotModel robotModel;

      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 1);

      public CentroidalRobotEstimator(FloatingJoint rootJoint, FullRobotModel controllerRobotModel)
      {
         this.estimatedRootJoint = rootJoint;
         this.robotModel = controllerRobotModel;
         this.controllerRootJoint = robotModel.getRootJoint();
      }

      @Override
      public void initialize()
      {

      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return getClass().getSimpleName();
      }

      @Override
      public String getDescription()
      {
         return "Estimator for the centroidal dynamics robot";
      }

      @Override
      public void doControl()
      {
         estimatedRootJoint.getPositionAndVelocity(position, velocity);
         estimatedRootJoint.getLinearAccelerationInWorld(acceleration);
         estimatedRootJoint.getRotationToWorld(orientation);
         estimatedRootJoint.getAngularVelocityInBody(angularVelocity);
         estimatedRootJoint.getAngularAccelerationInBody(angularAcceleration);

         controllerRootJoint.setPosition(position);
         controllerRootJoint.setRotation(orientation);
         tempMatrix.reshape(6, 1);
         angularVelocity.get(0, tempMatrix);
         velocity.get(3, tempMatrix);
         controllerRootJoint.setVelocity(tempMatrix, 0);
      }
   }

   public class CentroidalRobotJointMap implements JointNameMap<RobotCentroidal>
   {

      @Override
      public LegJointName[] getLegJointNames()
      {
         return null;
      }

      @Override
      public ArmJointName[] getArmJointNames()
      {
         return null;
      }

      @Override
      public SpineJointName[] getSpineJointNames()
      {
         return null;
      }

      @Override
      public NeckJointName[] getNeckJointNames()
      {
         return null;
      }

      @Override
      public String getModelName()
      {
         return robotName;
      }

      @Override
      public JointRole getJointRole(String jointName)
      {
         return null;
      }

      @Override
      public NeckJointName getNeckJointName(String jointName)
      {
         return null;
      }

      @Override
      public SpineJointName getSpineJointName(String jointName)
      {
         return null;
      }

      @Override
      public String getRootBodyName()
      {
         return null;
      }

      @Override
      public String getUnsanitizedRootJointInSdf()
      {
         return null;
      }

      @Override
      public String getHeadName()
      {
         return null;
      }

      @Override
      public boolean isTorqueVelocityLimitsEnabled()
      {
         return false;
      }

      @Override
      public Set<String> getLastSimulatedJoints()
      {
         return null;
      }

      @Override
      public String[] getJointNamesBeforeFeet()
      {
         return null;
      }

      @Override
      public RobotCentroidal[] getRobotSegments()
      {
         return null;
      }

      @Override
      public RobotCentroidal getEndEffectorsRobotSegment(String jointNameBeforeEndEffector)
      {
         return null;
      }
   }

   public class CentroidalRobotPhysicalProperties implements RobotPhysicalProperties
   {
      public CentroidalRobotPhysicalProperties()
      {
         
      }

      @Override
      public SideDependentList<RigidBodyTransform> getHandAttachmentPlateToWristTransforms()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public SideDependentList<RigidBodyTransform> getSoleToAnkleFrameTransforms()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public double getThighLength()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getShinLength()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getActualFootLength()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getActualFootWidth()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getFootForwardForControl()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getFootBackForControl()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getFootLengthForControl()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getToeWidthForControl()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getFootWidthForControl()
      {
         // TODO Auto-generated method stub
         return 0;
      }
   }
   
   public class CentroidalRobotDescription extends RobotDescription
   {
      public CentroidalRobotDescription(String namePrefix)
      {
         super(namePrefix);
         FloatingJointDescription rootJoint = new FloatingJointDescription(namePrefix + "RootJoint");
         LinkDescription rootLink = new LinkDescription(namePrefix + "RootLink");
         LinkGraphicsDescription rootLinkGraphics = new LinkGraphicsDescription();
         rootLinkGraphics.addEllipsoid(xRadius, yRadius, zRadius, new YoAppearanceRGBColor(Color.BLUE, 0.5));
         rootLink.setLinkGraphics(rootLinkGraphics);
         rootLink.setMass(robotMass);
         rootLink.setMomentOfInertia(momentOfInertia);
         addRootJoint(rootJoint);
         rootJoint.setLink(rootLink);
      }
   }
}

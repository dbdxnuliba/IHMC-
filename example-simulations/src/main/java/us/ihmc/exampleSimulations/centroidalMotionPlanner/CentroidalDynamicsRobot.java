package us.ihmc.exampleSimulations.centroidalMotionPlanner;

import java.awt.Color;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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
      CentroidalRobotEstimator estimator = new CentroidalRobotEstimator(floatingRootJointRobot.getRootJoint(), graphicsListRegistry);
      estimator.setStateToUpdate(controller.getState());
      floatingRootJointRobot.setController(estimator);
      floatingRootJointRobot.setController(controller);
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

   public abstract class CentroidalRobotPhysicalProperties
   {
      public abstract double getMass();

      public abstract double getNominalHeight();

      public abstract SideDependentList<ConvexPolygon2D> getDefaultSupportPolygons();

      public abstract DenseMatrix64F getInertia();
   }

   public class CentroidalRobotDescription extends RobotDescription
   {
      public CentroidalRobotDescription(double mass, double Ixx, double Ixy, double Ixz, double Iyy, double Iyz, double Izz)
      {
         this(mass, new DenseMatrix64F(3, 3, true, Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz));
      }

      public CentroidalRobotDescription(double mass, double Ixx, double Iyy, double Izz)
      {
         this(mass, Ixx, 0.0, 0.0, Iyy, 0.0, Izz);
      }

      public CentroidalRobotDescription(CentroidalRobotPhysicalProperties physicalProperties)
      {
         this(physicalProperties.getMass(), physicalProperties.getInertia());
      }

      public CentroidalRobotDescription(double mass, DenseMatrix64F inertiaTensor)
      {
         super("CentroidalBot");
         FloatingJointDescription rootJoint = new FloatingJointDescription("RootJoint");
         LinkDescription rootLink = new LinkDescription("RootLink");
         LinkGraphicsDescription rootLinkGraphics = new LinkGraphicsDescription();
         rootLinkGraphics.addEllipsoid(0.025, 0.025, 0.025, new YoAppearanceRGBColor(Color.BLUE, 0.5));
         rootLink.setLinkGraphics(rootLinkGraphics);
         rootLink.setMass(mass);
         rootLink.setMomentOfInertia(inertiaTensor);
         rootJoint.setLink(rootLink);
         addRootJoint(rootJoint);
      }
   }

   public class CentroidalRobotEstimator implements RobotController
   {
      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final YoVariableRegistry registry = new YoVariableRegistry("EstimatorRegistry");
      private final FloatingJoint rootJoint;
      private CentroidalRobotState state;
      private final YoFramePose pose;
      private final PoseReferenceFrame poseFrame;
      private final String namePrefix = "estimated";

      // Variables for cals 
      private final FramePoint3D tempPoint = new FramePoint3D();
      private final FrameVector3D tempVector = new FrameVector3D();
      private final FrameQuaternion tempQuaternion = new FrameQuaternion();

      public CentroidalRobotEstimator(FloatingJoint rootJoint, YoGraphicsListRegistry graphicsListRegistry)
      {
         this.rootJoint = rootJoint;
         pose = new YoFramePose(namePrefix + "Pose", worldFrame, registry);
         poseFrame = new PoseReferenceFrame(namePrefix + "Pose", worldFrame);
         if (graphicsListRegistry != null)
         {
            YoGraphicCoordinateSystem poseGraphic = new YoGraphicCoordinateSystem(namePrefix + "PoseGraphic", pose, 0.1);
            YoFramePoint2d yoPoint = new YoFramePoint2d(pose.getYoX(), pose.getYoY(), pose.getReferenceFrame());
            YoArtifactPosition positionArtifact = new YoArtifactPosition(namePrefix, yoPoint, GraphicType.BALL_WITH_CROSS, Color.BLACK, 0.002);
            graphicsListRegistry.registerYoGraphic("EstimatorGraphicsList", poseGraphic);
            graphicsListRegistry.registerArtifact("EstimatorArtifactList", positionArtifact);
         }
      }

      public void setStateToUpdate(CentroidalRobotState stateToUpdate)
      {
         this.state = stateToUpdate;
      }

      @Override
      public void initialize()
      {
         if(state == null)
            throw new RuntimeException("State object to update has not been set");
         doControl();
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return "CentroidalRobotEstimator";
      }

      @Override
      public String getDescription()
      {
         return "Provides an estimation of the robot's current pose";
      }

      @Override
      public void doControl()
      {
         rootJoint.getPosition(tempPoint);
         rootJoint.getQuaternion(tempQuaternion);
         poseFrame.setPoseAndUpdate(tempPoint, tempQuaternion);
         pose.set(tempPoint, tempQuaternion);

         state.setPosition(tempPoint);
         state.setOrientation(tempQuaternion);

         rootJoint.getVelocity(tempVector);
         state.setLinearVelocity(tempVector);

         rootJoint.getLinearAcceleration(tempVector);
         state.setLinearAcceleration(tempVector);

         rootJoint.getAngularVelocity(tempVector, poseFrame);
         tempVector.changeFrame(worldFrame);
         state.setAngularVelocity(tempVector);

         rootJoint.getAngularAcceleration(tempVector, poseFrame);
         tempVector.changeFrame(worldFrame);
         state.setAngularAcceleration(tempVector);
      }

      public CentroidalRobotState getState()
      {
         return state;
      }

      public void getState(CentroidalRobotState stateToSet)
      {
         stateToSet.set(state);
      }
   }

   public abstract class CentroidalRobotController implements RobotController
   {
      protected final YoVariableRegistry registry = new YoVariableRegistry("ControllerRegistry");
      private final CentroidalRobotState state;
      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final ExternalForcePoint forcePoint;
      private final YoFramePoint cop;
      private final YoFrameVector force;

      // Variables for setting and getting 
      private final FramePoint3D tempPoint = new FramePoint3D();
      private final FrameVector3D tempVector = new FrameVector3D();

      public CentroidalRobotController(String namePrefix, Joint joint, CentroidalRobotPhysicalProperties physicalProperties,
                                       YoGraphicsListRegistry graphicsListRegistry)
      {
         forcePoint = new ExternalForcePoint("controllerForcePoint", registry);
         state = new CentroidalRobotState(namePrefix + "State", registry);
         force = new YoFrameVector(namePrefix + "Force", worldFrame, registry);
         cop = new YoFramePoint(namePrefix + "CoP", worldFrame, registry);
         joint.addExternalForcePoint(forcePoint);
         if (graphicsListRegistry != null)
         {
            YoGraphicVector forceGraphic = new YoGraphicVector(namePrefix + "ForceGraphic", cop.getYoX(), cop.getYoY(), cop.getYoZ(), force.getYoX(),
                                                               force.getYoY(), force.getYoZ(), 1.0, new YoAppearanceRGBColor(Color.RED, 0.0), true);
            YoGraphicPosition copGraphic = new YoGraphicPosition(namePrefix + "CoPGraphic", cop, 1.0, new YoAppearanceRGBColor(Color.ORANGE, 0.0));
            graphicsListRegistry.registerArtifact("ControllerForceArtifact", copGraphic.createArtifact());
            graphicsListRegistry.registerYoGraphic("ControllerForceGraphic", forceGraphic);
            graphicsListRegistry.registerYoGraphic("ControllerForceGraphic", copGraphic);
         }
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return "CentroidalRobotController";
      }

      @Override
      public String getDescription()
      {
         return "Exerts a force from an external point on the root joint of robot";
      }

      public void finalizeControlInputs()
      {
         forcePoint.setOffsetWorld(cop.getX(), cop.getY(), cop.getZ());
         forcePoint.setForce(force.getX(), force.getY(), force.getZ());
      }

      public void setCoP(FramePoint3DReadOnly copToSet)
      {
         tempPoint.setIncludingFrame(copToSet);
         tempPoint.changeFrame(worldFrame);
         cop.set(tempPoint);
      }

      public void setForce(FrameVector3DReadOnly forceToSet)
      {
         tempVector.setIncludingFrame(forceToSet);
         tempVector.changeFrame(worldFrame);
         force.set(tempVector);
      }

      public CentroidalRobotState getState()
      {
         return state;
      }
   }

   public class CentroidalRobotState
   {
      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final YoFramePoint position;
      private final YoFrameVector linearVelocity;
      private final YoFrameVector linearAcceleration;
      private final YoFrameQuaternion orientation;
      private final YoFrameVector angularVelocity;
      private final YoFrameVector angularAcceleration;

      public CentroidalRobotState(String namePrefix, YoVariableRegistry registry)
      {
         position = new YoFramePoint(namePrefix + "Position", worldFrame, registry);
         linearVelocity = new YoFrameVector(namePrefix + "LinearVelocity", worldFrame, registry);
         linearAcceleration = new YoFrameVector(namePrefix + "LinearAcceleration", worldFrame, registry);
         orientation = new YoFrameQuaternion(namePrefix + "Orientation", worldFrame, registry);
         angularVelocity = new YoFrameVector(namePrefix + "AngularVelocity", worldFrame, registry);
         angularAcceleration = new YoFrameVector(namePrefix + "AngularAccleration", worldFrame, registry);
      }

      public void set(CentroidalRobotState other)
      {
         position.set(other.position);
         linearVelocity.set(other.linearVelocity);
         linearAcceleration.set(other.linearAcceleration);
         orientation.set(other.orientation);
         angularVelocity.set(other.angularVelocity);
         angularAcceleration.set(other.angularAcceleration);
      }

      public void getPosition(FramePoint3D positionToSet)
      {
         positionToSet.setIncludingFrame(position);
      }

      public void setPosition(FramePoint3D positionToSet)
      {
         position.set(positionToSet);
      }

      public void getLinearVelocity(FrameVector3D linearVelocityToSet)
      {
         linearVelocityToSet.setIncludingFrame(linearVelocity);
      }

      public void setLinearVelocity(FrameVector3D linearVelocityToSet)
      {
         linearVelocity.set(linearVelocityToSet);
      }

      public void getLinearAcceleration(FrameVector3D linearAccelerationToSet)
      {
         linearAccelerationToSet.setIncludingFrame(linearAcceleration);
      }

      public void setLinearAcceleration(FrameVector3D linearAccelerationToSet)
      {
         linearAcceleration.set(linearAccelerationToSet);
      }

      public void getOrientation(FrameQuaternion orientationToSet)
      {
         orientationToSet.setIncludingFrame(orientation);
      }

      public void setOrientation(FrameQuaternion orientationToSet)
      {
         orientation.set(orientationToSet);
      }

      public void getAngularVelocity(FrameVector3D angularVelocityToSet)
      {
         angularVelocityToSet.setIncludingFrame(angularVelocity);
      }

      public void setAngularVelocity(FrameVector3D angularVelocityToSet)
      {
         angularVelocity.set(angularVelocityToSet);
      }

      public void getAngularAcceleration(FrameVector3D angularAccelerationToSet)
      {
         angularAccelerationToSet.setIncludingFrame(angularAcceleration);
      }

      public void setAngularAcceleration(FrameVector3D angularAccelerationToSet)
      {
         angularAcceleration.set(angularAccelerationToSet);
      }
   }
}
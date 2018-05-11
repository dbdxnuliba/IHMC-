package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * <p>Implements a SQP based centroidal motion planning approach to enable a larger variety of motions 
 * This models the robot as a point mass and a contact surface that can exert a restricted force on the
 * point mass. Constraints are imposed on plan generated to ensure that the force computed acts collinear 
 * to the CoP (constrained to be within the contact surface) and CoM. This ensures that the plan generated 
 * does not result in angular momentum changes. </p>
 * Essentially the mathematical equation being solved is <b>xddot = u (x - &thetasym;) + g</b> where <b> x </b> is the CoM location,
 * <i> u  </i> is a scalar, <b>&thetasym;</b> is the CoP and <b> g </b> is the acceleration due to gravity
 * <p> All motion planning is done in the {@code ReferenceFrame#getWorldFrame()} </p>
 * @author Apoorv S
 *
 */
public class CollinearForceBasedCoMMotionPlanner
{
   public static final int numberOfScalarTrajectoryCoefficients = 4;
   public static final int numberOfCoMTrajectoryCoefficients = 8;
   public static final int numberOfCoPTrajectoryCoefficients = 8;
   private static final ReferenceFrame woldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   private final YoInteger maxNumberOfSQPIterations;
   private final YoDouble maxPlannerSegmentTime;
   private final YoInteger maxNumberOfPlanningSegments;
   private final YoDouble consolidatedConvergenceThreshold;
   private final YoDouble individualAxisConvergenceThreshold;

   private final YoInteger numberOfElapsedSQPIterations;
   private final YoBoolean hasPlanConverged;
   private final YoBoolean hasPlannerFailed;

   private final YoFramePoint initialCoMLocation;
   private final YoFramePoint initialCoPLocation;
   private final YoFrameVector initialCoMVelocity;
   private final YoFramePoint finalCoMLocation;
   private final YoFramePoint finalCoPLocation;
   private final YoFrameVector finalCoMVelocity;

   private final RecyclingArrayList<ContactState> motionPlannerNodeList;
   private final CollinearForceBasedPlannerIterationResult currentSolution;
   private final FrameVector3DReadOnly gravity;

   public CollinearForceBasedCoMMotionPlanner(FrameVector3DReadOnly gravity, YoVariableRegistry parentRegistry)
   {
      String namePrefix = getClass().getSimpleName();
      registry = new YoVariableRegistry(namePrefix);
      hasPlanConverged = new YoBoolean(namePrefix + "HasPlannerConverged", registry);
      hasPlannerFailed = new YoBoolean(namePrefix + "HasPlannerFailed", registry);
      numberOfElapsedSQPIterations = new YoInteger(namePrefix + "NumberOfElapsedSQPIterations", registry);

      maxNumberOfSQPIterations = new YoInteger(namePrefix + "MaxNumberOfSQPIterations", registry);
      maxPlannerSegmentTime = new YoDouble(namePrefix + "MaxPlannerSegmentTime", registry);
      maxNumberOfPlanningSegments = new YoInteger(namePrefix + "MaxNumberOfPlanningSegments", registry);

      consolidatedConvergenceThreshold = new YoDouble(namePrefix + "ConsolidatedConvergenceThreshold", registry);
      individualAxisConvergenceThreshold = new YoDouble(namePrefix + "IndividualAxisConvergenceThreshold", registry);

      initialCoMLocation = new YoFramePoint(namePrefix + "InitialCoMLocation", woldFrame, registry);
      initialCoPLocation = new YoFramePoint(namePrefix + "InitialCoPLocation", woldFrame, registry);
      initialCoMVelocity = new YoFrameVector(namePrefix + "InitialCoMVelocity", woldFrame, registry);
      finalCoMLocation = new YoFramePoint(namePrefix + "FinalCoMLocation", woldFrame, registry);
      finalCoPLocation = new YoFramePoint(namePrefix + "FinalCoPLocation", woldFrame, registry);
      finalCoMVelocity = new YoFrameVector(namePrefix + "FinalCoMVelocity", woldFrame, registry);

      motionPlannerNodeList = new RecyclingArrayList<>(100, ContactState.class);
      currentSolution = new CollinearForceBasedPlannerIterationResult(gravity);
      this.gravity = gravity;
      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize(CollinearForcePlannerParameters parameters)
   {
      maxNumberOfSQPIterations.set(parameters.getMaxSQPIterations());
      maxPlannerSegmentTime.set(parameters.getMaxPlannerSegmentTime());
      maxNumberOfPlanningSegments.set(parameters.getMaxNumberOfPlanningSegments());
      consolidatedConvergenceThreshold.set(parameters.getConsolidatedConvergenceThreshold());
      individualAxisConvergenceThreshold.set(parameters.getIndividualAxisConvergenceThreshold());
   }

   public void reset()
   {
      numberOfElapsedSQPIterations.set(0);
      motionPlannerNodeList.clear();
   }

   public void setInitialState(FramePoint3D initialCoMLocation, FrameVector3D initialCoMVelocity, FramePoint3D initialCoPLocation)
   {
      this.initialCoMLocation.set(initialCoMLocation);
      this.initialCoMVelocity.set(initialCoMVelocity);
      this.initialCoPLocation.set(initialCoPLocation);
   }

   public void setFinalState(FramePoint3D finalCoMLocation, FrameVector3D finalCoMVelocity, FrameVector3D finalCoPLocation)
   {
      this.finalCoMLocation.set(finalCoMLocation);
      this.finalCoMVelocity.set(finalCoMVelocity);
      this.finalCoPLocation.set(finalCoPLocation);
   }

   public void submitContactSequenceNode(ContactState contactState)
   {
      if (isNodeValid(contactState))
      {
         motionPlannerNodeList.add().set(contactState);
      }
   }

   private boolean checkIfSQPHasConverged()
   {
      double totalDynamicsViolation = 0.0;
      if (!currentSolution.didIterationConverge())
         return false;
      for (Axis axis : Axis.values)
      {
         double axisViolation = Math.abs(currentSolution.getViolation(axis));
         totalDynamicsViolation += axisViolation;
         if (axisViolation > individualAxisConvergenceThreshold.getDoubleValue())
            return false;
      }
      if (totalDynamicsViolation > consolidatedConvergenceThreshold.getDoubleValue())
         return false;
      return true;
   }

   private boolean isNodeValid(ContactState contactStateToCheck)
   {
      if (contactStateToCheck.getDuration() <= 0.0f)
         return false;
      if (contactStateToCheck.getContactType() == null)
         return false;
      if (contactStateToCheck.getReferenceFrame() == null)
         return false;
      if (contactStateToCheck.getContactType().isRobotSupported() && contactStateToCheck.getNumberOfSupportPolygonVertices() <= 0)
         return false;
      return true;
   }

   public void runIterations(int numberOfSQPIterationsToRun)
   {

   }

   public boolean hasPlanConverged()
   {
      return false;
   }

   public boolean hasPlannerFailed()
   {
      return false;
   }
}

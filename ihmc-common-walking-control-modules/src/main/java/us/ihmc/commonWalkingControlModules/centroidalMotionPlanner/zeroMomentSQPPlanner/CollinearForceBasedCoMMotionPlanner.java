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
 * Essentially the mathematical equation being solved is <b>xddot = u (x - &n;) + g</b> where <b> x </b> is the CoM location,
 * <i> u  </i> is a scalar, <b>&nu;</b> is the CoP and <b> g </b> is the acceleration due to gravity
 * <p> All motion planning is done in the {@code ReferenceFrame#getWorldFrame()} </p>
 * @author Apoorv S
 *
 */
public class CollinearForceBasedCoMMotionPlanner
{
   public static final int numberOfScalarTrajectoryCoefficients = 4;
   public static final int numberOfCoMTrajectoryCoefficients = 8;
   public static final int numberOfCoPTrajectoryCoefficients = 8;

   private final YoVariableRegistry registry;
   private final YoInteger maxNumberOfSQPIterations;
   private final YoDouble consolidatedConvergenceThreshold;
   private final YoDouble individualAxisConvergenceThreshold;

   private final YoInteger numberOfElapsedSQPIterations;
   private final YoBoolean hasPlanConverged;
   private final YoBoolean hasPlannerFailed;

   private final CollinearForceBasedPlannerIterationResult sqpSolution;
   private final CollinearForceBasedPlannerOptimizationControlModule sqpOptimizationControlModule;

   public CollinearForceBasedCoMMotionPlanner(FrameVector3DReadOnly gravity, YoVariableRegistry parentRegistry)
   {
      String namePrefix = getClass().getSimpleName();
      registry = new YoVariableRegistry(namePrefix);
      hasPlanConverged = new YoBoolean(namePrefix + "HasPlannerConverged", registry);
      hasPlannerFailed = new YoBoolean(namePrefix + "HasPlannerFailed", registry);
      numberOfElapsedSQPIterations = new YoInteger(namePrefix + "NumberOfElapsedSQPIterations", registry);

      maxNumberOfSQPIterations = new YoInteger(namePrefix + "MaxNumberOfSQPIterations", registry);

      consolidatedConvergenceThreshold = new YoDouble(namePrefix + "ConsolidatedConvergenceThreshold", registry);
      individualAxisConvergenceThreshold = new YoDouble(namePrefix + "IndividualAxisConvergenceThreshold", registry);

      sqpSolution = new CollinearForceBasedPlannerIterationResult(gravity);
      sqpOptimizationControlModule = new CollinearForceBasedPlannerOptimizationControlModule(sqpSolution, gravity, registry);
      parentRegistry.addChild(registry);
      reset();
   }

   public void initialize(CollinearForcePlannerParameters parameters)
   {
      maxNumberOfSQPIterations.set(parameters.getMaxSQPIterations());
      consolidatedConvergenceThreshold.set(parameters.getConsolidatedConvergenceThreshold());
      individualAxisConvergenceThreshold.set(parameters.getIndividualAxisConvergenceThreshold());
   }

   public void reset()
   {
      hasPlanConverged.set(false);
      hasPlannerFailed.set(false);
      numberOfElapsedSQPIterations.set(0);
   }

   public void setInitialState(FramePoint3D initialCoMLocation, FrameVector3D initialCoMVelocity, FramePoint3D initialCoPLocation)
   {
      sqpOptimizationControlModule.setInitialState(initialCoMLocation, initialCoMVelocity, initialCoPLocation);
   }

   public void setFinalState(FramePoint3D finalCoMLocation, FrameVector3D finalCoMVelocity, FrameVector3D finalCoPLocation)
   {
      sqpOptimizationControlModule.setFinalState(finalCoMLocation, finalCoMVelocity, finalCoPLocation);
   }

   public void submitContactSequenceNode(ContactState contactState)
   {
      if (isNodeValid(contactState))
      {
         sqpOptimizationControlModule.submitContactState(contactState);
      }
   }

   private boolean checkIsDynamicsViolationIsBelowThresholds()
   {
      double totalDynamicsViolation = 0.0;
      if (!sqpSolution.didIterationConverge())
         return false;
      for (Axis axis : Axis.values)
      {
         double axisViolation = Math.abs(sqpSolution.getViolation(axis));
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
      for (int i = 0; i < numberOfSQPIterationsToRun; i++)
      {
         if (!hasPlanConverged() && !hasPlannerFailed())
         {
            sqpOptimizationControlModule.generateLinearizedSystemObjective();
            sqpOptimizationControlModule.generateLinearizedSystemConstraints();
            if(!sqpOptimizationControlModule.compute())
            {
               hasPlannerFailed.set(true);
               break;
            }
            sqpOptimizationControlModule.updateSolution();
            hasPlanConverged.set(checkIsDynamicsViolationIsBelowThresholds());
            numberOfElapsedSQPIterations.increment();
            hasPlannerFailed.set(numberOfElapsedSQPIterations.getIntegerValue() > maxNumberOfSQPIterations.getIntegerValue());
         }
         else
            break;
      }
   }

   public boolean hasPlanConverged()
   {
      return hasPlanConverged.getBooleanValue();
   }

   public boolean hasPlannerFailed()
   {
      return hasPlannerFailed.getBooleanValue();
   }
}

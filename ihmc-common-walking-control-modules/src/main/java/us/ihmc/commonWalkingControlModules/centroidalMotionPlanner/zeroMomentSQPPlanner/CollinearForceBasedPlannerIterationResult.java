package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;

/**
 * Stores the output of the {@code CollinearForceBasedCoMMotionPlanner}. This can be used to 
 * generate commands for the {@code WholeBodyControllerCore} and also as a seed for the next
 * SQP iteration.
 * 
 * @author Apoorv S
 *
 */
public class CollinearForceBasedPlannerIterationResult
{
   private static final int numberOfCoefficientsForComputedAccelerationTrajectory = Math.max(CollinearForceBasedCoMMotionPlanner.numberOfCoMTrajectoryCoefficients,
                                                                                             CollinearForceBasedCoMMotionPlanner.numberOfCoPTrajectoryCoefficients)
         + CollinearForceBasedCoMMotionPlanner.numberOfScalarTrajectoryCoefficients - 1;
   private final static int defaultNumberOfSegments = 100;

   ReferenceFrame referenceFrame;
   int iterationCount;
   boolean qpConvergenceFlag;

   final RecyclingArrayList<Trajectory3D> comTrajectories;
   final RecyclingArrayList<Trajectory3D> copTrajectories;
   final RecyclingArrayList<Trajectory> scalarProfile;

   private final FramePoint3D comPosition = new FramePoint3D();
   private final FramePoint3D copPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D comAcceleration = new FrameVector3D();
   private final FrameVector3DReadOnly gravity;

   private TrajectoryMathTools trajectoryMathToolbox = new TrajectoryMathTools(numberOfCoefficientsForComputedAccelerationTrajectory);
   private final Trajectory accelerationFromDifferentiation = new Trajectory(CollinearForceBasedCoMMotionPlanner.numberOfCoMTrajectoryCoefficients - 2);
   private final Trajectory accelerationFromDynamics = new Trajectory(numberOfCoefficientsForComputedAccelerationTrajectory);
   private final Trajectory tempTrajectory = new Trajectory(numberOfCoefficientsForComputedAccelerationTrajectory);
   private final Trajectory dynamicsErrorTrajectory = new Trajectory(numberOfCoefficientsForComputedAccelerationTrajectory);

   public CollinearForceBasedPlannerIterationResult(FrameVector3DReadOnly gravity)
   {
      comTrajectories = new RecyclingArrayList<>(defaultNumberOfSegments, new GenericTypeBuilder<Trajectory3D>()
      {

         @Override
         public Trajectory3D newInstance()
         {
            return new Trajectory3D(CollinearForceBasedCoMMotionPlanner.numberOfCoMTrajectoryCoefficients);
         }

      });
      copTrajectories = new RecyclingArrayList<>(defaultNumberOfSegments, new GenericTypeBuilder<Trajectory3D>()
      {

         @Override
         public Trajectory3D newInstance()
         {
            return new Trajectory3D(CollinearForceBasedCoMMotionPlanner.numberOfCoPTrajectoryCoefficients);
         }
      });
      scalarProfile = new RecyclingArrayList<>(defaultNumberOfSegments, new GenericTypeBuilder<Trajectory>()
      {

         @Override
         public Trajectory newInstance()
         {
            return new Trajectory(CollinearForceBasedCoMMotionPlanner.numberOfScalarTrajectoryCoefficients);
         }

      });
      this.gravity = gravity;
      reset();
   }

   public void reset()
   {
      qpConvergenceFlag = false;
      iterationCount = -1;
      comTrajectories.clear();
      copTrajectories.clear();
      scalarProfile.clear();
   }

   public void compute(double timeInState)
   {
      int currentSegmentIndex = getCurrentSegmentFromTime(timeInState);
      if (currentSegmentIndex < 0)
         throw new RuntimeException("Unable to find segment associated with the provided time in state");
      Trajectory3D currentCoMTrajectory = comTrajectories.get(currentSegmentIndex);
      Trajectory3D currentCoPTrajectory = copTrajectories.get(currentSegmentIndex);
      currentCoMTrajectory.compute(timeInState);
      comPosition.setIncludingFrame(referenceFrame, currentCoMTrajectory.getPosition());
      copPosition.setIncludingFrame(referenceFrame, currentCoPTrajectory.getPosition());
      double scalarValue = scalarProfile.get(currentSegmentIndex).getPosition();
      comVelocity.setIncludingFrame(referenceFrame, currentCoMTrajectory.getVelocity());
      comAcceleration.changeFrame(referenceFrame);
      comAcceleration.sub(comPosition, copPosition);
      comAcceleration.scale(scalarValue);
      comAcceleration.add(gravity);
   }

   public FramePoint3D getDesiredCoMPosition()
   {
      return comPosition;
   }

   public FrameVector3D getDesiredCoMVelocity()
   {
      return comVelocity;
   }

   public FramePoint3D getDesiredCoPPosition()
   {
      return copPosition;
   }

   public FrameVector3D getDesiredCoMAcceleration()
   {
      return comAcceleration;
   }

   private int getCurrentSegmentFromTime(double timeInState)
   {
      for (int i = 0; i < comTrajectories.size(); i++)
      {
         if (comTrajectories.get(i).getInitialTime() < timeInState && comTrajectories.get(i).getFinalTime() >= timeInState)
            return i;
      }
      return -1;
   }

   public double getViolation(Axis axis)
   {
      double axisViolation = 0.0;
      for (int i = 0; i < comTrajectories.size(); i++)
      {
         TrajectoryMathTools.subtract(tempTrajectory, comTrajectories.get(i).getTrajectory(axis), copTrajectories.get(i).getTrajectory(axis));
         trajectoryMathToolbox.multiply(accelerationFromDynamics, tempTrajectory, scalarProfile.get(i));
         accelerationFromDynamics.setDirectly(0, accelerationFromDynamics.getCoefficient(0) + gravity.getElement(axis.ordinal()));
         comTrajectories.get(i).getTrajectory(axis).getDerivative(accelerationFromDifferentiation, 2);
         TrajectoryMathTools.subtract(dynamicsErrorTrajectory, accelerationFromDifferentiation, accelerationFromDynamics);
         double axisSegmentViolation = dynamicsErrorTrajectory.getIntegral(dynamicsErrorTrajectory.getInitialTime(), dynamicsErrorTrajectory.getFinalTime());
         axisViolation += axisSegmentViolation;
      }
      return axisViolation;
   }

   public boolean didIterationConverge()
   {
      return qpConvergenceFlag;
   }
}

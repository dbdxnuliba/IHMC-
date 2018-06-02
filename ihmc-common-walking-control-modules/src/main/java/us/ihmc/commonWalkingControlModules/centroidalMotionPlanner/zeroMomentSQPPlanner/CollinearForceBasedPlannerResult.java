package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Stores the output of the {@code CollinearForceBasedCoMMotionPlanner}. This can be used to 
 * generate commands for the {@code WholeBodyControllerCore} and also as a seed for the next
 * SQP iteration.
 * 
 * @author Apoorv S
 *
 */
public class CollinearForceBasedPlannerResult
{
   private static final int numberOfCoefficientsForComputedAccelerationTrajectory = Math.max(CollinearForceBasedCoMMotionPlanner.numberOfCoMTrajectoryCoefficients,
                                                                                             CollinearForceBasedCoMMotionPlanner.numberOfCoPTrajectoryCoefficients)
         + CollinearForceBasedCoMMotionPlanner.numberOfScalarTrajectoryCoefficients - 1;
   private final static int defaultNumberOfSegments = 100;

   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
   int iterationCount;
   boolean qpConvergenceFlag;

   final RecyclingArrayList<Trajectory3D> comTrajectories;
   final RecyclingArrayList<Trajectory3D> copTrajectories;
   final RecyclingArrayList<Trajectory> scalarProfile;

   private final FramePoint3D comPosition = new FramePoint3D();
   private final FramePoint3D copPosition = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();
   private final FrameVector3D comAcceleration = new FrameVector3D();
   private final FrameVector3D groundReactionForce = new FrameVector3D();
   private FrameVector3DReadOnly gravity;
   private final YoFramePoint yoCoMPosition;
   private final YoFramePoint yoCoPPosition;
   private final YoFrameVector yoCoMVelocity;
   private final YoFrameVector yoCoMAccelerationDifference;
   private final YoFrameVector yoCoMDynamicsAcceleration;
   private final YoFrameVector yoCoMDesiredAcceleration;
   private final YoFrameVector yoGroundReactionForce;
   private final YoInteger yoCurrentSegmentIndex;
   private final YoDouble yoScalar;
   private final YoDouble mass;

   private TrajectoryMathTools trajectoryMathToolbox = new TrajectoryMathTools(numberOfCoefficientsForComputedAccelerationTrajectory);
   private final Trajectory accelerationFromDifferentiation = new Trajectory(CollinearForceBasedCoMMotionPlanner.numberOfCoMTrajectoryCoefficients - 2);
   private final Trajectory accelerationFromDynamics = new Trajectory(numberOfCoefficientsForComputedAccelerationTrajectory);
   private final Trajectory tempTrajectory = new Trajectory(numberOfCoefficientsForComputedAccelerationTrajectory);
   private final Trajectory dynamicsErrorTrajectory = new Trajectory(numberOfCoefficientsForComputedAccelerationTrajectory);

   public CollinearForceBasedPlannerResult(YoVariableRegistry registry)
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
      mass = new YoDouble("Mass", registry);
      String namePrefix = "SQPOutput";
      yoCurrentSegmentIndex = new YoInteger(namePrefix + "SegmentIndex", registry);
      yoCoMPosition = new YoFramePoint(namePrefix + "CoMPosition", referenceFrame, registry);
      yoCoPPosition = new YoFramePoint(namePrefix + "CoPPosition", referenceFrame, registry);
      yoCoMVelocity = new YoFrameVector(namePrefix + "CoMVelocity", referenceFrame, registry);
      yoCoMDynamicsAcceleration = new YoFrameVector(namePrefix + "CoMDynamicsAcceleration", referenceFrame, registry);
      yoCoMDesiredAcceleration = new YoFrameVector(namePrefix + "CoMDesiredAcceleration", referenceFrame, registry);
      yoCoMAccelerationDifference = new YoFrameVector(namePrefix +"CoMAccelerationDifference", referenceFrame, registry);
      yoGroundReactionForce = new YoFrameVector(namePrefix + "GroundReactionForce", referenceFrame, registry);
      yoScalar = new YoDouble(namePrefix + "Scalar", registry);
      reset();
   }

   public void reset()
   {
      qpConvergenceFlag = false;
      iterationCount = 0;
      comTrajectories.clear();
      copTrajectories.clear();
      scalarProfile.clear();
   }

   public void compute(double timeInState)
   {
      int currentSegmentIndex = getCurrentSegmentFromTime(timeInState);
      if (currentSegmentIndex < 0)
      {
         //currentSegmentIndex = comTrajectories.size() - 1;
         //timeInState = comTrajectories.get(currentSegmentIndex).getFinalTime() + getSegmentStartTime(currentSegmentIndex);
         throw new RuntimeException("Unable to find segment associated with the provided time in state " + timeInState);
      }
      yoCurrentSegmentIndex.set(currentSegmentIndex);
      Trajectory3D currentCoMTrajectory = comTrajectories.get(currentSegmentIndex);
      Trajectory3D currentCoPTrajectory = copTrajectories.get(currentSegmentIndex);
      Trajectory currentScalarTrajectory = scalarProfile.get(currentSegmentIndex);
      double segmentStartTime = getSegmentStartTime(currentSegmentIndex);
      currentCoMTrajectory.compute(timeInState - segmentStartTime);
      currentCoPTrajectory.compute(timeInState - segmentStartTime);
      currentScalarTrajectory.compute(timeInState - segmentStartTime);
      comPosition.setIncludingFrame(referenceFrame, currentCoMTrajectory.getPosition());
      copPosition.setIncludingFrame(referenceFrame, currentCoPTrajectory.getPosition());
      double scalarValue = currentScalarTrajectory.getPosition();
      comVelocity.setIncludingFrame(referenceFrame, currentCoMTrajectory.getVelocity());

      groundReactionForce.changeFrame(referenceFrame);
      groundReactionForce.sub(comPosition, copPosition);
      groundReactionForce.scale(scalarValue);
      comAcceleration.setIncludingFrame(groundReactionForce);
      comAcceleration.add(gravity);
      groundReactionForce.scale(mass.getDoubleValue());
      
      yoGroundReactionForce.set(groundReactionForce);
      yoCoMPosition.set(comPosition);
      yoCoMVelocity.set(comVelocity);
      yoCoMDynamicsAcceleration.set(comAcceleration);
      yoCoMDesiredAcceleration.set(currentCoMTrajectory.getAcceleration());
      yoCoMAccelerationDifference.sub(yoCoMDesiredAcceleration, yoCoMDynamicsAcceleration);
      yoCoPPosition.set(copPosition);
      yoScalar.set(scalarValue);
   }

   private double getSegmentStartTime(int currentSegmentIndex)
   {
      double startTime = 0.0;
      for (int i = 0; i < currentSegmentIndex; i++)
         startTime += comTrajectories.get(i).getFinalTime();
      return startTime;
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return yoCoMPosition;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return yoCoMVelocity;
   }

   public FramePoint3DReadOnly getDesiredCoPPosition()
   {
      return yoCoPPosition;
   }

   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return yoCoMDesiredAcceleration;
   }

   public FrameVector3DReadOnly getDynamicsCoMAcceleration()
   {
      return yoCoMDynamicsAcceleration;
   }

   private int getCurrentSegmentFromTime(double timeInState)
   {
      double segmentStartTime = 0.0;
      for (int i = 0; i < comTrajectories.size(); i++)
      {
         double initialTime = comTrajectories.get(i).getInitialTime() + segmentStartTime;
         double finalTime = comTrajectories.get(i).getFinalTime() + segmentStartTime;
         if (initialTime <= timeInState && finalTime >= timeInState)
            return i;
         segmentStartTime = finalTime;
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

   public String toString()
   {
      String ret = "";
      for (int i = 0; i < comTrajectories.size(); i++)
      {
         ret += "Segment: " + i + "\n";
         ret += "CoM: " + comTrajectories.get(i).toString() + "\n";
         ret += "CoP: " + copTrajectories.get(i).toString() + "\n";
         ret += "Scalar: " + scalarProfile.get(i).toString() + "\n";
      }
      return ret;
   }

   public FrameVector3DReadOnly getDesiredGroundReactionForce()
   {
      return groundReactionForce;
   }

   public void initialize(FrameVector3DReadOnly gravity, double mass)
   {
      this.gravity = gravity;
      this.mass.set(mass);
   }

   public double getScalar()
   {
      return yoScalar.getDoubleValue();
   }
}

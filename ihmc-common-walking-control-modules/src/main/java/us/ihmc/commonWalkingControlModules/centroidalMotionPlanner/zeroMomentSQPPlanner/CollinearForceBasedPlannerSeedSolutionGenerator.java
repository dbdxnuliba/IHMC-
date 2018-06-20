package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

/**
 * Generates a seed solution for the SQP solver. The solution generated may not be dynamically feasible 
 * and/or smooth / continuous.
 * @author Apoorv S
 *
 */
public class CollinearForceBasedPlannerSeedSolutionGenerator
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FramePoint3D initialCoMPosition = new FramePoint3D();
   private final FramePoint3D finalCoMPosition = new FramePoint3D();
   private final FramePoint3D initialCoPPosition = new FramePoint3D();
   private final FramePoint3D finalCoPPosition = new FramePoint3D();
   private final FrameVector3D initialCoMVelocity = new FrameVector3D();
   private final FrameVector3D finalCoMVelocity = new FrameVector3D();
   private RecyclingArrayList<CollinearForceMotionPlannerSegment> segmentList;

   private CollinearForceBasedPlannerResult sqpSolution;
   private FrameVector3DReadOnly gravity;
   private final YoFramePoint3D comNominalOffsetFromSupportPolygonCentroid;
   private final Point3D tempPoint = new Point3D();
   private final FramePoint3D tempFramePoint = new FramePoint3D();

   public CollinearForceBasedPlannerSeedSolutionGenerator(YoVariableRegistry registry)
   {
      String namePrefix = getClass().getSimpleName();
      comNominalOffsetFromSupportPolygonCentroid = new YoFramePoint3D(namePrefix + "NominalCoMOffsetFromSupportPolygonCentroid", worldFrame, registry);
   }

   /**
    * Specifies the data structure in which the seed solution is to be saved
    * @param seedResultToSet
    */
   void initialize(CollinearForceBasedPlannerResult seedResultToSet, FrameVector3DReadOnly gravity, CollinearForcePlannerParameters parameters)
   {
      gravity.checkReferenceFrameMatch(worldFrame);
      this.gravity = gravity;
      this.sqpSolution = seedResultToSet;
      Point3DReadOnly nominalCoMOffset = parameters.getNominalCoMOffsetFromSupportPolygonCentroid();
      if (nominalCoMOffset.getZ() < 1e-3)
         throw new RuntimeException("Invalid nominal CoM offset above support surface");
      comNominalOffsetFromSupportPolygonCentroid.set(nominalCoMOffset);
   }

   public void reset()
   {
      segmentList = null;
      initialCoMPosition.setToNaN();
      initialCoPPosition.setToNaN();
      initialCoMVelocity.setToNaN();
      finalCoMPosition.setToNaN();
      finalCoPPosition.setToNaN();
      finalCoMVelocity.setToNaN();
   }

   public void submitSegmentList(RecyclingArrayList<CollinearForceMotionPlannerSegment> segmentList)
   {
      this.segmentList = segmentList;
   }

   public void setInitialState(FramePoint3DReadOnly initialCoMPosition, FramePoint3DReadOnly initialCoPPosition, FrameVector3DReadOnly initialCoMVelocity)
   {
      this.initialCoMPosition.setIncludingFrame(initialCoMPosition);
      this.initialCoPPosition.setIncludingFrame(initialCoPPosition);
      this.initialCoMVelocity.setIncludingFrame(initialCoMVelocity);
   }

   public void setFinalState(FramePoint3DReadOnly finalCoMPosition, FramePoint3DReadOnly finalCoPPosition, FrameVector3DReadOnly finalCoMVelocity)
   {
      this.finalCoMPosition.setIncludingFrame(finalCoMPosition);
      this.finalCoPPosition.setIncludingFrame(finalCoPPosition);
      this.finalCoMVelocity.setIncludingFrame(finalCoMVelocity);
   }

   public void computeSeedSolution()
   {
      sqpSolution.reset();
      RecyclingArrayList<Trajectory3D> comTrajectories = sqpSolution.comTrajectories;
      RecyclingArrayList<Trajectory3D> copTrajectories = sqpSolution.copTrajectories;
      RecyclingArrayList<Trajectory> scalarProfile = sqpSolution.scalarProfile;
      for (int i = 0; i < segmentList.size(); i++)
      {
         CollinearForceMotionPlannerSegment segment = segmentList.get(i);
         Trajectory3D comTrajectory = comTrajectories.add();
         Trajectory3D copTrajectory = copTrajectories.add();
         Trajectory scalarTrajectory = scalarProfile.add();
         segment.getSupportPolygonCentroid(tempPoint);
         if(tempFramePoint.containsNaN())
            tempFramePoint.setToZero(worldFrame);
         tempFramePoint.changeFrame(worldFrame);
         tempFramePoint.add(comNominalOffsetFromSupportPolygonCentroid.getX(), comNominalOffsetFromSupportPolygonCentroid.getY(),
                            comNominalOffsetFromSupportPolygonCentroid.getZ());
         setConstant(comTrajectory, CollinearForceBasedCoMMotionPlanner.numberOfCoMTrajectoryCoefficients, segment.getSegmentDuration(), tempFramePoint);
         tempFramePoint.sub(0.0, 0.0, comNominalOffsetFromSupportPolygonCentroid.getZ());
         setConstant(copTrajectory, CollinearForceBasedCoMMotionPlanner.numberOfCoPTrajectoryCoefficients, segment.getSegmentDuration(), tempFramePoint);
         if (segment.isSupported())
            setConstant(scalarTrajectory, CollinearForceBasedCoMMotionPlanner.numberOfScalarTrajectoryCoefficients, segment.getSegmentDuration(),
                        (0.0 - gravity.getZ()) / (comNominalOffsetFromSupportPolygonCentroid.getZ()));
         else
            setConstant(scalarTrajectory, CollinearForceBasedCoMMotionPlanner.numberOfScalarTrajectoryCoefficients, segment.getSegmentDuration(), 0.0);
      }
   }

   private void setConstant(Trajectory3D trajectory, int numberOfCoefficients, double duration, FramePoint3D constantPoint)
   {
      for (Axis axis : Axis.values)
         setConstant(trajectory.getTrajectory(axis), numberOfCoefficients, duration, constantPoint.getElement(axis.ordinal()));
   }

   private void setConstant(Trajectory trajectory, int numberOfCoefficients, double duration, double value)
   {
      trajectory.setInitialTime(0.0);
      trajectory.setFinalTime(duration);
      trajectory.setDirectly(0, value);
      for (int i = 1; i < numberOfCoefficients; i++)
         trajectory.setDirectly(i, 0.0);
   }
}
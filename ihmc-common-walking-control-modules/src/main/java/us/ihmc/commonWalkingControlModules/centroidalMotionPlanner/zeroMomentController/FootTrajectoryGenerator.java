package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootTrajectoryGenerator
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RobotSide robotSide;
   private final YoDouble trajectoryStartTime;
   private final YoDouble trajectoryTime;
   private final FootTrajectory trajectory;

   private final YoDouble nominalFirstSegmentPercentage;
   private final YoDouble nominalLastSegmentPercentage;
   private final YoFrameVector defaultFinalVelocity;
   private final YoFrameVector defaultFinalAcceleration;
   private final YoDouble intermediatePointHeightAboveGround;
   private double duration;
   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialVelocity = new FrameVector3D();
   private final FrameVector3D initialAcceleration = new FrameVector3D();
   private final FramePoint3D intermediatePosition1 = new FramePoint3D();
   private final FramePoint3D intermediatePosition2 = new FramePoint3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalVelocity = new FrameVector3D();
   private final FrameVector3D finalAcceleration = new FrameVector3D();

   public FootTrajectoryGenerator(RobotSide side, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      String namePrefix = side.getCamelCaseNameForStartOfExpression() + "Foot";
      robotSide = side;
      trajectory = new FootTrajectory(side.getCamelCaseNameForStartOfExpression(), registry);
      trajectoryStartTime = new YoDouble(namePrefix + "InitialTrajectoryTime", registry);
      trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      nominalFirstSegmentPercentage = new YoDouble(namePrefix + "NominalFirstSegmentPercentrage", registry);
      nominalLastSegmentPercentage = new YoDouble(namePrefix + "NominalLastSegmentPercentage", registry);
      intermediatePointHeightAboveGround = new YoDouble(namePrefix + "NominalHeightAboveGround", registry);
      defaultFinalVelocity = new YoFrameVector(namePrefix + "DefaultFinalVelocity", worldFrame, registry);
      defaultFinalAcceleration = new YoFrameVector(namePrefix + "DefaultFinalAcceleration", worldFrame, registry);
   }

   public void initialize(double nominalFirstSegmentPercentageDuration, double nominalLastSegmentPercentageDuration, FrameVector3DReadOnly defaultFinalVelocity,
                          FrameVector3DReadOnly defaultFinalAcceleration, double nominalHeightAboveGround)
   {
      this.nominalFirstSegmentPercentage.set(nominalFirstSegmentPercentageDuration);
      this.nominalLastSegmentPercentage.set(nominalLastSegmentPercentageDuration);
      this.defaultFinalVelocity.set(defaultFinalVelocity);
      this.defaultFinalAcceleration.set(defaultFinalAcceleration);
      this.intermediatePointHeightAboveGround.set(nominalHeightAboveGround);
   }

   public void updateTrajectory(double time)
   {
      trajectoryStartTime.set(time);
      computeIntermediatePositionHeuristics(initialPosition, finalPosition);
      double t1 = duration * nominalFirstSegmentPercentage.getDoubleValue();
      double t2 = duration * (1.0 - nominalLastSegmentPercentage.getDoubleValue());
      trajectory.setInitialConditions(0.0, initialPosition, initialVelocity, initialAcceleration);
      trajectory.setIntermediateLocations(t1, intermediatePosition1, t2, intermediatePosition2);
      trajectory.setFinalConditions(duration, finalPosition, finalVelocity, finalAcceleration);
      trajectory.initialize();
   }

   public void compute(double time, FramePoint3D desiredPositionToSet, FrameVector3D desiredVelocityToSet, FrameVector3D desiredAccelerationToSet)
   {
      compute(time);
      trajectory.getDesireds(desiredPositionToSet, desiredVelocityToSet, desiredAccelerationToSet);
      if(isDone())
      {
         desiredVelocityToSet.setToZero();
         desiredAccelerationToSet.setToZero();
      }
   }

   public void compute(double time)
   {
      trajectoryTime.set(time - trajectoryStartTime.getDoubleValue());
      trajectory.compute(trajectoryTime.getDoubleValue());
   }

   public boolean isDone()
   {
      return trajectoryTime.getDoubleValue() >= trajectory.getFinalTime();
   }

   public void setInitialConditions(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity, FrameVector3DReadOnly initialAcceleration)
   {
      this.initialPosition.setIncludingFrame(initialPosition);
      this.initialPosition.changeFrame(worldFrame);
      this.initialVelocity.setIncludingFrame(initialVelocity);
      this.initialVelocity.changeFrame(worldFrame);
      this.initialAcceleration.setIncludingFrame(initialAcceleration);
      this.initialAcceleration.changeFrame(worldFrame);
   }

   public void setFinalConditions(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity, FrameVector3DReadOnly finalAcceleration)
   {
      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalPosition.changeFrame(worldFrame);
      this.finalVelocity.setIncludingFrame(finalVelocity);
      this.finalVelocity.changeFrame(worldFrame);
      this.finalAcceleration.setIncludingFrame(finalAcceleration);
      this.finalAcceleration.changeFrame(worldFrame);
   }

   public void setFinalConditions(FramePoint3DReadOnly finalPosition)
   {
      this.finalPosition.setIncludingFrame(finalPosition);
      this.finalPosition.changeFrame(worldFrame);
      this.finalVelocity.setIncludingFrame(defaultFinalVelocity);
      this.finalAcceleration.setIncludingFrame(defaultFinalAcceleration);
   }

   public void setDuration(double duration)
   {
      this.duration = duration;
   }
   
   private void computeIntermediatePositionHeuristics(FramePoint3DReadOnly initialPosition, FramePoint3DReadOnly finalPosition)
   {
      intermediatePosition1.setAndScale(1.0 - nominalFirstSegmentPercentage.getDoubleValue(), initialPosition);
      intermediatePosition1.scaleAdd(nominalFirstSegmentPercentage.getDoubleValue(), finalPosition, intermediatePosition1);
      intermediatePosition1.addZ(intermediatePointHeightAboveGround.getDoubleValue());
      intermediatePosition2.setAndScale(nominalLastSegmentPercentage.getDoubleValue(), initialPosition);
      intermediatePosition2.scaleAdd(1.0 - nominalLastSegmentPercentage.getDoubleValue(), finalPosition, intermediatePosition2);
      intermediatePosition2.addZ(intermediatePointHeightAboveGround.getDoubleValue());
   }

   public FramePoint3DReadOnly getPosition()
   {
      return trajectory.getPosition();
   }
}

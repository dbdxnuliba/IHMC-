package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Wrapper around {@code FrameTrajectory3D} to enable required variable logging
 * @author Apoorv S
 */
public class FootTrajectory
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoDouble t0;
   private final YoDouble t1;
   private final YoDouble t2;
   private final YoDouble tF;
   private final YoFrameVector defaultTouchdownVelocity;
   private final YoFrameVector defaultTouchdownAcceleration;

   private final YoFramePoint initialPosition;
   private final YoFrameVector initialVelocity;
   private final YoFrameVector initialAcceleration;
   private final YoFramePoint intermediatePosition1;
   private final YoFramePoint intermediatePosition2;
   private final YoFramePoint finalPosition;
   private final YoFrameVector finalVelocity;
   private final YoFrameVector finalAcceleration;
   private final YoInteger currentSegmentIndex;

   private final ArrayList<FrameTrajectory3D> segments = new ArrayList<>();
   private final FrameVector3D tempInitialVelocity = new FrameVector3D();
   private final FrameVector3D tempInitialAcceleration = new FrameVector3D();
   private final FrameVector3D tempFinalVelocity = new FrameVector3D();
   private final FrameVector3D tempFinalAcceleration = new FrameVector3D();

   public FootTrajectory(String footName, YoVariableRegistry registry)
   {
      String namePrefix = footName + "Trajectory";
      this.currentSegmentIndex = new YoInteger(namePrefix + "SegmentIndex", registry);
      this.t0 = new YoDouble(namePrefix + "InitialTime", registry);
      this.t1 = new YoDouble(namePrefix + "IntermediateTime1", registry);
      this.t2 = new YoDouble(namePrefix + "IntermediateTime2", registry);
      this.tF = new YoDouble(namePrefix + "FinalTime", registry);
      this.initialPosition = new YoFramePoint(namePrefix + "InitialPosition", worldFrame, registry);
      this.initialVelocity = new YoFrameVector(namePrefix + "IntialVelocity", worldFrame, registry);
      this.initialAcceleration = new YoFrameVector(namePrefix + "InitialAcceleration", worldFrame, registry);
      this.intermediatePosition1 = new YoFramePoint(namePrefix + "IntermediatePoint1", worldFrame, registry);
      this.intermediatePosition2 = new YoFramePoint(namePrefix + "IntermediatePoint2", worldFrame, registry);
      this.finalPosition = new YoFramePoint(namePrefix + "FinalPosition", worldFrame, registry);
      this.finalVelocity = new YoFrameVector(namePrefix + "FinalVelocity", worldFrame, registry);
      this.finalAcceleration = new YoFrameVector(namePrefix + "FinalAcceleration", worldFrame, registry);

      this.defaultTouchdownVelocity = new YoFrameVector(namePrefix + "DefaultTouchdownVelocity", worldFrame, registry);
      this.defaultTouchdownAcceleration = new YoFrameVector(namePrefix + "DefaultTouchdownAcceleration", worldFrame, registry);

      segments.add(new FrameTrajectory3D(4, worldFrame));
      segments.add(new FrameTrajectory3D(6, worldFrame));
      segments.add(new FrameTrajectory3D(4, worldFrame));
      reset();
   }

   public void reset()
   {
      t0.set(Double.NaN);
      t1.set(Double.NaN);
      t2.set(Double.NaN);
      tF.set(Double.NaN);
      for (int i = 0; i < segments.size(); i++)
         segments.get(i).reset();
   }

   public void setDefaultTouchdownDesireds(FrameVector3DReadOnly touchdownVelocity, FrameVector3DReadOnly touchdownAcceleration)
   {
      this.defaultTouchdownVelocity.set(touchdownVelocity);
      this.defaultTouchdownAcceleration.set(touchdownAcceleration);
   }

   public void setInitialConditions(double t0, FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialVelocity,
                                    FrameVector3DReadOnly initialAcceleration)
   {
      this.t0.set(t0);
      this.initialPosition.set(initialPosition);
      this.initialVelocity.set(initialVelocity);
      this.initialAcceleration.set(initialAcceleration);
   }

   public void setIntermediateLocations(double t1, FramePoint3DReadOnly intermediatePosition1, double t2, FramePoint3DReadOnly intermediatePosition2)
   {
      this.t1.set(t1);
      this.intermediatePosition1.set(intermediatePosition1);
      this.t2.set(t2);
      this.intermediatePosition2.set(intermediatePosition2);
   }

   public void setFinalConditions(double tF, FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalVelocity, FrameVector3DReadOnly finalAcceleration)
   {
      this.tF.set(tF);
      this.finalPosition.set(finalPosition);
      this.finalVelocity.set(finalVelocity);
      this.finalAcceleration.set(finalAcceleration);
   }

   public void setFinalConditions(double tF, FramePoint3DReadOnly finalPosition)
   {
      this.tF.set(tF);
      this.finalPosition.set(finalPosition);
      useDefaultFinalVelocityAndAcceleration();
   }

   public void initialize()
   {
      FrameTrajectory3D firstSegment = segments.get(0);
      double t0 = this.t0.getDoubleValue();
      double t1 = this.t1.getDoubleValue();
      double t2 = this.t2.getDoubleValue();
      double tF = this.tF.getDoubleValue();
      firstSegment.setCubicThreeInitialConditionsFinalPosition(t0, t1, initialPosition, initialVelocity, initialAcceleration, intermediatePosition1);
      firstSegment.compute(t1);
      firstSegment.getFrameVelocity(tempInitialVelocity);
      firstSegment.getFrameAcceleration(tempInitialAcceleration);

      FrameTrajectory3D lastSegment = segments.get(2);
      lastSegment.setCubicInitialPositionThreeFinalConditions(t2, tF, intermediatePosition2, finalPosition, finalVelocity, finalAcceleration);
      lastSegment.compute(t2);
      lastSegment.getFrameVelocity(tempFinalVelocity);
      lastSegment.getFrameAcceleration(tempFinalAcceleration);

      segments.get(1).setQuintic(t1, t2, initialVelocity, tempInitialVelocity, tempInitialAcceleration, intermediatePosition2, tempFinalVelocity,
                                 tempFinalAcceleration);
   }

   public void compute(double time)
   {
      double clippedTime = updateCurrentSegmentIndex(time);
      segments.get(currentSegmentIndex.getIntegerValue()).compute(clippedTime);
   }

   private double updateCurrentSegmentIndex(double time)
   {
      int index = 0;
      for (; index < segments.size(); index++)
         if (time < segments.get(index).getFinalTime())
            break;
      if (index == segments.size())
      {
         currentSegmentIndex.set(index - 1);
         time = segments.get(index - 1).getFinalTime();
      }
      else
         currentSegmentIndex.set(index);
      return time;
   }

   public void getDesireds(FramePoint3D desiredPositionToSet, FrameVector3D desiredVelocityToSet, FrameVector3D desiredAccelerationToSet)
   {
      FrameTrajectory3D currentSegment = segments.get(currentSegmentIndex.getIntegerValue());
      desiredPositionToSet.setIncludingFrame(currentSegment.getFramePosition());
      desiredVelocityToSet.setIncludingFrame(currentSegment.getFrameVelocity());
      desiredAccelerationToSet.setIncludingFrame(currentSegment.getFrameAcceleration());
   }

   private void useDefaultFinalVelocityAndAcceleration()
   {
      finalVelocity.set(defaultTouchdownVelocity);
      finalAcceleration.set(defaultTouchdownAcceleration);
   }

   public double getInitialTime()
   {
      return t0.getDoubleValue();
   }

   public double getFinalTime()
   {
      return tF.getDoubleValue();
   }

   public FramePoint3DReadOnly getPosition()
   {
      return segments.get(currentSegmentIndex.getIntegerValue()).getFramePosition();
   }

   public FrameTuple3DReadOnly getVelocity()
   {
      return segments.get(currentSegmentIndex.getIntegerValue()).getFrameVelocity();
   }

   public FrameTuple3DReadOnly getAcceleration()
   {
      return segments.get(currentSegmentIndex.getIntegerValue()).getFrameAcceleration();
   }
}

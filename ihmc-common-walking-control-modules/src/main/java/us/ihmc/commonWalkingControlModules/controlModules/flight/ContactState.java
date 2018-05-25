package us.ihmc.commonWalkingControlModules.controlModules.flight;

import com.jme3.animation.Pose;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

/**
 * Stores the contact state of a robot in 2.5D representation
 * @author Apoorv
 *
 */
public class ContactState implements ReferenceFrameHolder
{
   /**
    * The duration of this contact state
    */
   private double duration;
   /**
    * The pose with respect to which the support polygon is defined
    */
   private final FramePose3D pose;
   /**
    * The support polygon during the this contact state
    */
   private final ConvexPolygon2D supportPolygon;

   public ContactState()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public ContactState(ReferenceFrame referenceFrame)
   {
      this.pose = new FramePose3D(referenceFrame);
      this.supportPolygon = new ConvexPolygon2D();
      reset();
   }

   public void reset()
   {
      duration = Double.NaN;
      supportPolygon.clear();
      pose.setToZero();
   }

   /**
    * Sets the support polygon 
    * @param supportPolygonToSet should be defined wrt to the specified pose 
    */
   public void setSupportPolygon(ConvexPolygon2D supportPolygonToSet)
   {
      this.supportPolygon.set(supportPolygonToSet);
   }

   /**
    * Sets the support polygon
    * @param supportPolygonToSet support polygon to set. Will be stored wrt to the specified pose
    */
   public void setSupportPolygon(FrameConvexPolygon2d supportPolygonToSet)
   {
      supportPolygon.set(supportPolygonToSet.getGeometryObject());
      TransformHelperTools.transformFromReferenceFrameToReferenceFrame(supportPolygonToSet.getReferenceFrame(), getReferenceFrame(), supportPolygon);
      TransformHelperTools.transformFromReferenceFrameToPose(pose, supportPolygon);
   }

   public void setDuration(double duration)
   {
      this.duration = duration;
   }

   /**
    * Returns the support polygon wrt to the specified {@code ReferenceFrame}
    * @param referenceFrame
    * @param supportPolygonToSet
    */
   public void getSupportPolygon(ReferenceFrame referenceFrame, FrameConvexPolygon2d supportPolygonToSet)
   {
      supportPolygonToSet.setIncludingFrame(getReferenceFrame(), supportPolygon);
      TransformHelperTools.transformFromPoseToReferenceFrame(pose, supportPolygonToSet.getGeometryObject());
      supportPolygonToSet.changeFrame(referenceFrame);
   }

   /**
    * Returns the support polygon wrt to the contact state pose
    * @param supportPolygonToSet
    */
   public void getSupportPolygon(ConvexPolygon2D supportPolygonToSet)
   {
      supportPolygonToSet.set(supportPolygon);
   }

   public boolean isSupported()
   {
      return supportPolygon.getNumberOfVertices() > 0;
   }

   public double getDuration()
   {
      return duration;
   }

   public void setPose(FramePose3DReadOnly poseToSet)
   {
      pose.setIncludingFrame(poseToSet);
   }

   public void setPose(FramePose2DReadOnly poseToSet)
   {
      pose.setIncludingFrame(poseToSet);
   }

   public FramePose3DReadOnly getPose()
   {
      return pose;
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(pose.getPosition());
   }

   public void setPosition(FramePoint3DReadOnly positionToSet)
   {
      pose.setPosition(positionToSet);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(this.pose.getOrientation());
   }

   public void setOrientation(FrameQuaternionReadOnly orientationToSet)
   {
      this.pose.setOrientation(orientationToSet);
   }

   public void getSupportPolygonCentroid(FramePoint3D framePointToSet)
   {
      framePointToSet.setIncludingFrame(getReferenceFrame(), supportPolygon.getCentroid(), 0.0);
      TransformHelperTools.transformFromPoseToReferenceFrame(pose, framePointToSet);
   }

   public void set(ContactState other)
   {
      this.duration = other.duration;
      this.pose.setIncludingFrame(other.pose);
      this.supportPolygon.set(other.supportPolygon);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return pose.getReferenceFrame();
   }

   public int getNumberOfSupportPolygonVertices()
   {
      return supportPolygon.getNumberOfVertices();
   }

   public String toString()
   {
      String toString = "isSupported: " + isSupported() + ",\nDuration: " + duration + ",\nPose: " + pose.toString() + ",\nSupportPolygon: "
            + supportPolygon.toString();
      return toString;
   }

   public void setPoseToZero(ReferenceFrame referenceFrame)
   {
      pose.setToZero(referenceFrame);
   }
}
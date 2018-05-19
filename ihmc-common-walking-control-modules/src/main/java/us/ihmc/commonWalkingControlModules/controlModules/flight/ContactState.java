package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;

/**
 * Stores the centroidal state of a robot in 2.5D representation
 * @author Apoorv
 *
 */
public class ContactState implements ReferenceFrameHolder
{
   private BipedContactType state;
   private double duration;
   private final FramePose3D pose;
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
      state = null;
      duration = Double.NaN;
      supportPolygon.clear();
      pose.setToZero();
   }

   public void setContactType(BipedContactType stateToSet)
   {
      this.state = stateToSet;
   }

   public void setSupportPolygon(ConvexPolygon2D supportPolygonToSet)
   {
      this.supportPolygon.set(supportPolygonToSet);
   }

   public void setDuration(double duration)
   {
      this.duration = duration;
   }

   public void getSupportPolygon(ConvexPolygon2D supportPolygonToSet)
   {
      supportPolygonToSet.set(supportPolygon);
   }

   public BipedContactType getContactType()
   {
      return state;
   }

   public double getDuration()
   {
      return duration;
   }

   public void setSupportPolygonFramePose(FramePose3D poseToSet)
   {
      pose.setIncludingFrame(poseToSet);
   }

   public FramePose3D getSupportPolygonFramePose()
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

   public void getSupportPolygonCentroid(FramePoint3D tempFramePoint)
   {
      tempFramePoint.setIncludingFrame(pose.getReferenceFrame(), supportPolygon.getCentroid(), 0.0);
   }

   public void set(ContactState other)
   {
      this.state = other.state;
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
      String toString = "SupportType: " + state.toString() + ",\nDuration: " + duration + ",\nPose: " + pose.toString() + ",\nSupportPolygon: "
            + supportPolygon.toString();
      return toString;
   }
}
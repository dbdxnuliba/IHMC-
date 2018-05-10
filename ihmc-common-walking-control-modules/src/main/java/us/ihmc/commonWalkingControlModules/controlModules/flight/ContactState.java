package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Stores the centroidal state of a robot in 2.5D representation
 * @author Apoorv
 *
 */
public class ContactState
{
   private BipedContactType state;
   private double duration;
   // TODO check if there is a better way to represent the support polygon orientation
   private final FrameQuaternion orientation;
   private final ConvexPolygon2D supportPolygon;

   public ContactState()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public ContactState(ReferenceFrame referenceFrame)
   {
      this.orientation = new FrameQuaternion(referenceFrame);
      this.supportPolygon = new ConvexPolygon2D();
      reset();
   }

   public void reset()
   {
      state = null;
      duration = Double.NaN;
      supportPolygon.clear();
      orientation.setToZero();
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

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(this.orientation);
   }

   public void setOrientation(FrameQuaternion orientationToSet)
   {
      this.orientation.setIncludingFrame(orientationToSet);
   }
}
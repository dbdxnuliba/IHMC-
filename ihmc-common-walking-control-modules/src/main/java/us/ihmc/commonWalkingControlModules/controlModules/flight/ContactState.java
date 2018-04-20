package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class ContactState
{
   private ContactType state;
   private double duration;
   private final FrameConvexPolygon2d supportPolygon;

   public ContactState()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public ContactState(ReferenceFrame referenceFrame)
   {
      this.supportPolygon = new FrameConvexPolygon2d(referenceFrame);
      reset();
   }

   public void reset()
   {
      state = null;
      duration = Double.NaN;
      supportPolygon.clear();
   }

   public void setContactType(ContactType stateToSet)
   {
      this.state = stateToSet;
   }

   public void setSupportPolygonIncludingFrame(FrameConvexPolygon2d supportPolygonToSet)
   {
      this.supportPolygon.setIncludingFrame(supportPolygonToSet);
   }

   public void setSupportPolygon(FrameConvexPolygon2d supportPolygonToSet)
   {
      this.supportPolygon.set(supportPolygonToSet);
   }

   public void setDuration(double duration)
   {
      this.duration = duration;
   }

   public void getSupportPolygon(FrameConvexPolygon2d supportPolygonToSet)
   {
      supportPolygonToSet.setIncludingFrame(supportPolygon);
   }

   public ContactType getContactType()
   {
      return state;
   }

   public double getDuration()
   {
      return duration;
   }
}
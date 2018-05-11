package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import us.ihmc.commonWalkingControlModules.controlModules.flight.ContactState;

public class CollinearForceMotionPlannerSegment
{
   private double duration;
   private ContactState segmentContactState;
   private ContactState nextSegmentContactState;
   private boolean isContactStateChangingAfterSegment;
   
   public void setContactState(ContactState contactStateToSave)
   {
      this.segmentContactState = contactStateToSave;
   }
   
   public void setNextSegmentContactState(ContactState nextSegmentContactStateToSave)
   {
      this.nextSegmentContactState = nextSegmentContactStateToSave;
   }

   public void setContactStateChangeFlag(boolean isContactStateChanging)
   {
      this.isContactStateChangingAfterSegment = isContactStateChanging;
   }

   public ContactState getContactState()
   {
      return segmentContactState;
   }
   
   public ContactState getNextSegmentContactState()
   {
      return nextSegmentContactState;
   }

   public boolean isContactStateChangingAfterSegment()
   {
      return isContactStateChangingAfterSegment;
   }

   public void setSegmentDuration(double segmentDuration)
   {
      this.duration = segmentDuration;
   }

   public double getSegmentDuration()
   {
      return duration;
   }
}

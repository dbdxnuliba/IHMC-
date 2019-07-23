package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import controller_msgs.msg.dds.*;
import us.ihmc.euclid.interfaces.Settable;

public enum ControllerStatus
{
   CapturabilityBasedStatus              (CapturabilityBasedStatus                 .class),
   FootstepStatusMessage                 (FootstepStatusMessage                    .class),
   PlanOffsetStatus                      (PlanOffsetStatus                         .class),
   WalkingStatusMessage                  (WalkingStatusMessage                     .class),
   WalkingControllerFailureStatusMessage (WalkingControllerFailureStatusMessage    .class),
   ManipulationAbortedStatus             (ManipulationAbortedStatus                .class),
   HighLevelStateChangeStatusMessage     (HighLevelStateChangeStatusMessage        .class),
   TextToSpeechPacket                    (TextToSpeechPacket                       .class),
   ControllerCrashNotificationPacket     (ControllerCrashNotificationPacket        .class),
   JointspaceTrajectoryStatusMessage     (JointspaceTrajectoryStatusMessage        .class),
   TaskspaceTrajectoryStatusMessage      (TaskspaceTrajectoryStatusMessage         .class),
   ;

   public static final ControllerStatus[] values = values();
   private Class<? extends Settable<?>> statusClass;

   ControllerStatus(Class<? extends Settable<?>> statusClass)
   {
      this.statusClass = statusClass;
   }

   public Class<? extends Settable<?>> getStatusClass()
   {
      return statusClass;
   }
}

package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;

public enum ControllerCommand
{
   ArmTrajectoryCommand                           (ArmTrajectoryCommand                            .class),
   HandTrajectoryCommand                          (HandTrajectoryCommand                           .class),
   FootTrajectoryCommand                          (FootTrajectoryCommand                           .class),
   HeadTrajectoryCommand                          (HeadTrajectoryCommand                           .class),
   NeckTrajectoryCommand                          (NeckTrajectoryCommand                           .class),
   NeckDesiredAccelerationsCommand                (NeckDesiredAccelerationsCommand                 .class),
   ChestTrajectoryCommand                         (ChestTrajectoryCommand                          .class),
   SpineTrajectoryCommand                         (SpineTrajectoryCommand                          .class),
   PelvisTrajectoryCommand                        (PelvisTrajectoryCommand                         .class),
   PelvisOrientationTrajectoryCommand             (PelvisOrientationTrajectoryCommand              .class),
   PelvisHeightTrajectoryCommand                  (PelvisHeightTrajectoryCommand                   .class),
   StopAllTrajectoryCommand                       (StopAllTrajectoryCommand                        .class),
   FootstepDataListCommand                        (FootstepDataListCommand                         .class),
   AdjustFootstepCommand                          (AdjustFootstepCommand                           .class),
   GoHomeCommand                                  (GoHomeCommand                                   .class),
   FootLoadBearingCommand                         (FootLoadBearingCommand                          .class),
   ArmDesiredAccelerationsCommand                 (ArmDesiredAccelerationsCommand                  .class),
   AutomaticManipulationAbortCommand              (AutomaticManipulationAbortCommand               .class),
   HighLevelControllerStateCommand                (HighLevelControllerStateCommand                 .class),
   AbortWalkingCommand                            (AbortWalkingCommand                             .class),
   PrepareForLocomotionCommand                    (PrepareForLocomotionCommand                     .class),
   PauseWalkingCommand                            (PauseWalkingCommand                             .class),
   SpineDesiredAccelerationsCommand               (SpineDesiredAccelerationsCommand                .class),
   HandLoadBearingCommand                         (HandLoadBearingCommand                          .class),
   HandHybridJointspaceTaskspaceTrajectoryCommand (HandHybridJointspaceTaskspaceTrajectoryCommand  .class),
   HeadHybridJointspaceTaskspaceTrajectoryCommand (HeadHybridJointspaceTaskspaceTrajectoryCommand  .class),
   ChestHybridJointspaceTaskspaceTrajectoryCommand(ChestHybridJointspaceTaskspaceTrajectoryCommand .class),
   ClearDelayQueueCommand                         (ClearDelayQueueCommand                          .class),
   MomentumTrajectoryCommand                      (MomentumTrajectoryCommand                       .class),
   CenterOfMassTrajectoryCommand                  (CenterOfMassTrajectoryCommand                   .class),
   PlanarRegionsListCommand                       (PlanarRegionsListCommand                        .class),
   ;

   public static final ControllerCommand[] values = values();
   private Class<? extends Command<?, ?>> commandClass;

   ControllerCommand(Class<? extends Command<?, ?>> commandClass)
   {
      this.commandClass = commandClass;
   }

   public Class<? extends Command<?, ?>> getCommandClass()
   {
      return commandClass;
   }
}

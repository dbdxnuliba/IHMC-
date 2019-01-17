package us.ihmc.avatar.networkProcessor.footstepProcessingToolboxModule;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepProcessingRequestMessage;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FootstepProcessingToolboxController extends ToolboxController
{
   private final DistanceBasedSwingTimeCalculator swingTimeCalculator;

   FootstepProcessingToolboxController(FootstepProcessingParameters parameters, StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);
      this.swingTimeCalculator = new DistanceBasedSwingTimeCalculator(parameters.getMinimumSwingTime(),
                                                                      parameters.getMaximumStepTranslationForMinimumSwingTime(),
                                                                      parameters.getMaximumSwingTime(),
                                                                      parameters.getMinimumStepTranslationForMaximumSwingTime());
   }

   @Override
   public boolean initialize()
   {
      return true;
   }

   @Override
   public void updateInternal()
   {
   }

   void processMessage(FootstepProcessingRequestMessage requestMessage)
   {
      Point3D startPoint = new Point3D();
      Point3D endPoint = new Point3D();
      Object<FootstepDataMessage> footstepDataList = requestMessage.getFootstepDataListMessage().getFootstepDataList();

      for (int i = 0; i < footstepDataList.size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataList.get(i);
         endPoint.set(footstepDataMessage.getLocation());

         if(i < 2)
         {
            startPoint.set(footstepDataMessage.getRobotSide() == RobotSide.LEFT.toByte() ?
                               requestMessage.getLeftFootLocation() :
                               requestMessage.getRightFootLocation());
         }
         else
         {
            startPoint.set(footstepDataList.get(i - 2).getLocation());
         }

         footstepDataMessage.setSwingDuration(swingTimeCalculator.calculateSwingTime(startPoint, endPoint));
      }
   }

   @Override
   public boolean isDone()
   {
      // is always single-shot
      return true;
   }
}

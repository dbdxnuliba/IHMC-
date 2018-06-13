package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.motionController.states;

import us.ihmc.robotics.robotSide.RobotSide;

public enum MotionControllerStateEnum
{
   DOUBLE_SUPPORT, LEFT_SINGLE_SUPPORT, RIGHT_SINGLE_SUPPORT, NO_SUPPORT;

   public boolean isFootSupported(RobotSide side)
   {
      switch (this)
      {
      case DOUBLE_SUPPORT:
         return true;
      case LEFT_SINGLE_SUPPORT:
         return side == RobotSide.LEFT;
      case RIGHT_SINGLE_SUPPORT:
         return side == RobotSide.RIGHT;
      case NO_SUPPORT:
         return false;
      default:
         throw new RuntimeException("Unknown motion state");
      }
   }

   public boolean isLeftFootSupported()
   {
      return isFootSupported(RobotSide.LEFT);
   }

   public boolean isRightFootSuppported()
   {
      return isFootSupported(RobotSide.RIGHT);
   }
}

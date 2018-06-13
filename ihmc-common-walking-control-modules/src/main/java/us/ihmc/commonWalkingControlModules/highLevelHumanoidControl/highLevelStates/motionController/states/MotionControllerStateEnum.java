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

   public boolean canBeTerminalState()
   {
      switch (this)
      {
      case DOUBLE_SUPPORT:
      case LEFT_SINGLE_SUPPORT:
      case RIGHT_SINGLE_SUPPORT:
         return true;
      case NO_SUPPORT:
         return false;
      default:
         throw new RuntimeException("Unknown motion state");
      }
   }

   public String getAbbreviatedStateNameForMiddleOfExpression()
   {
      switch (this)
      {
      case DOUBLE_SUPPORT:
         return "DS";
      case RIGHT_SINGLE_SUPPORT:
         return "RSS";
      case LEFT_SINGLE_SUPPORT:
         return "LSS";
      case NO_SUPPORT:
         return "NS";
      default:
         throw new RuntimeException("Unknown motion state");
      }
   }

   public String getAbbreviatedStateNameForStartOfExpression()
   {
      switch (this)
      {
      case DOUBLE_SUPPORT:
         return "ds";
      case RIGHT_SINGLE_SUPPORT:
         return "rss";
      case LEFT_SINGLE_SUPPORT:
         return "lss";
      case NO_SUPPORT:
         return "ns";
      default:
         throw new RuntimeException("Unknown motion state");
      }
   }

   public static MotionControllerStateEnum getState(boolean isLeftFootSupported, boolean isRightFootSupported)
   {
      if(isLeftFootSupported && isRightFootSupported)
         return MotionControllerStateEnum.DOUBLE_SUPPORT;
      else if(isLeftFootSupported)
         return MotionControllerStateEnum.LEFT_SINGLE_SUPPORT;
      else if (isRightFootSupported)
         return MotionControllerStateEnum.RIGHT_SINGLE_SUPPORT;
      else
         return MotionControllerStateEnum.NO_SUPPORT;
   }
}

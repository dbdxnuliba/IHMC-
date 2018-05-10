package us.ihmc.commonWalkingControlModules.controlModules.flight;

import us.ihmc.robotics.robotSide.RobotSide;

public enum BipedContactType
{
   DOUBLE_SUPPORT, NO_SUPPORT, LEFT_SINGLE_SUPPORT, RIGHT_SINGLE_SUPPORT;

   public boolean isRobotSupported()
   {
      return isLeftFootSupported() || isRightFootSupported();
   }

   public boolean isLeftFootSupported()
   {
      switch (this)
      {
      case DOUBLE_SUPPORT:
      case LEFT_SINGLE_SUPPORT:
         return true;
      case RIGHT_SINGLE_SUPPORT:
      case NO_SUPPORT:
         return false;
      default:
         return throwUnhandledException();
      }
   }

   public boolean isRightFootSupported()
   {
      switch (this)
      {
      case DOUBLE_SUPPORT:
      case RIGHT_SINGLE_SUPPORT:
         return true;
      case LEFT_SINGLE_SUPPORT:
      case NO_SUPPORT:
         return false;
      default:
         return throwUnhandledException();
      }
   }

   public boolean areBothFeetSupported()
   {
      return isLeftFootSupported() && isRightFootSupported();
   }

   public boolean isOnlyOneFootSupported()
   {
      boolean leftFootSupported = isLeftFootSupported();
      boolean rightFootSupported = isRightFootSupported();
      return (leftFootSupported && !rightFootSupported) || (rightFootSupported && !leftFootSupported);
   }

   private boolean throwUnhandledException()
   {
      throw new RuntimeException("Unhandled case");
   }

   public boolean isFootSupported(RobotSide robotSide)
   {
      switch (robotSide)
      {
      case LEFT:
         return isLeftFootSupported();
      case RIGHT:
         return isRightFootSupported();
      default:
         return throwUnhandledException();
      }
   }
}
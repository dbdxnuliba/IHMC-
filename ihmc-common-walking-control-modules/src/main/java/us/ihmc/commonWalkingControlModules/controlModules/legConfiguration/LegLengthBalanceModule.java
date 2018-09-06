package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

import us.ihmc.yoVariables.variable.YoDouble;

public class LegLengthBalanceModule
{
   private double desiredAngle;

   public void compute()
   {
      desiredAngle=1.3;
   }
   public double getDesiredAngleWhenStraight()
   {
      return desiredAngle;
   }
}

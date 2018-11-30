package us.ihmc.valkyrie.parameters;

import us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance.HeightForBalanceParameters;

public class ValkyrieHeightForBalanceParameters extends HeightForBalanceParameters
{
   @Override
   public double getMaxHeightFirstPhaseOfSwing()
   {
      return 1.064;
   }

   @Override
   public double getMinHeight()
   {
      return 0.87;
   }
}

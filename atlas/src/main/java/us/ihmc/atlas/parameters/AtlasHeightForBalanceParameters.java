package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.capturePoint.heightForBalance.HeightForBalanceParameters;

public class AtlasHeightForBalanceParameters extends HeightForBalanceParameters
{
   @Override
   public double getMaxHeightFirstPhaseOfSwing()
   {
      return 1.17;
   }

   @Override
   public double getMinHeight()
   {
      return 1.0;
   }
}

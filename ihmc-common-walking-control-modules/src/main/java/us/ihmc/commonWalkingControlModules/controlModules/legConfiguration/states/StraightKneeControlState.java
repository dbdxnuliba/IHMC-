package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.states;

import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlToolbox;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains.LegConfigurationGainsReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class StraightKneeControlState implements LegControlState
{
   private final LegConfigurationControlToolbox toolbox;

   private final DoubleProvider desiredAngle;

   public StraightKneeControlState(LegConfigurationControlToolbox toolbox)
   {
      this.toolbox = toolbox;
      this.desiredAngle = toolbox.getDesiredStraightLegAngle();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return false;
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public LegConfigurationGainsReadOnly getLegConfigurationGains()
   {
      return toolbox.getStraightLegGains();
   }

   @Override
   public double getKneePitchPrivilegedConfiguration()
   {
      return desiredAngle.getValue();
   }
}

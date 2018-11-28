package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.states;

import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationControlToolbox;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains.LegConfigurationGainsReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoEnum;

public class BentKneeControlState implements LegControlState
{
   private final LegConfigurationControlToolbox toolbox;

   public BentKneeControlState(LegConfigurationControlToolbox toolbox)
   {
      this.toolbox = toolbox;
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
      toolbox.setLegControlWeight(LegConfigurationControlModule.LegControlWeight.LOW);
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public LegConfigurationGainsReadOnly getLegConfigurationGains()
   {
      return toolbox.getBentLegGains();
   }

   @Override
   public double getKneePitchPrivilegedConfiguration()
   {
      return toolbox.getKneeMidRangeOfMotion();
   }
}

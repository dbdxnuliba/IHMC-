package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.states;

import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains.LegConfigurationGainsReadOnly;
import us.ihmc.robotics.stateMachine.core.State;

public interface LegControlState extends State
{
   LegConfigurationGainsReadOnly getLegConfigurationGains();
   double getKneePitchPrivilegedConfiguration();
}

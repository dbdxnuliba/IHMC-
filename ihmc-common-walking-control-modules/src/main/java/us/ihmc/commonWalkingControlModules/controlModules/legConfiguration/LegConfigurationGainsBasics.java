package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

public interface LegConfigurationGainsBasics extends LegConfigurationGainsReadOnly
{
   void setJointSpaceKp(double kp);
   void setJointSpaceKd(double kd);
   void setActuatorSpaceKp(double kp);
   void setActuatorSpaceKd(double kd);
   void setSpringSpaceKp(double kp);
   void setSpringSpaceKd(double kd);
}

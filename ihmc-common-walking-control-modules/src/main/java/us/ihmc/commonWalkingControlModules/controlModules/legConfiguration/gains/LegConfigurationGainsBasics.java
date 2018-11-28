package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains;

public interface LegConfigurationGainsBasics extends LegConfigurationGainsReadOnly
{
   default void set(LegConfigurationGainsReadOnly other)
   {
      setJointSpaceKp(other.getJointSpaceKp());
      setJointSpaceKd(other.getJointSpaceKd());
      setActuatorSpaceKp(other.getActuatorSpaceKp());
      setActuatorSpaceKd(other.getActuatorSpaceKd());
      setSpringSpaceKp(other.getSpringSpaceKp());
      setSpringSpaceKd(other.getSpringSpaceKd());
   }

   void setJointSpaceKp(double kp);
   void setJointSpaceKd(double kd);
   void setActuatorSpaceKp(double kp);
   void setActuatorSpaceKd(double kd);
   void setSpringSpaceKp(double kp);
   void setSpringSpaceKd(double kd);
}

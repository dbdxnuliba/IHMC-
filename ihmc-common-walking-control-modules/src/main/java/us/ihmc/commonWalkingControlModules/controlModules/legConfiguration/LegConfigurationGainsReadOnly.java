package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration;

public interface LegConfigurationGainsReadOnly
{
   double getJointSpaceKp();
   double getJointSpaceKd();
   double getActuatorSpaceKp();
   double getActuatorSpaceKd();
   double getSpringSpaceKp();
   double getSpringSpaceKd();

   default boolean hasJointSpaceKp()
   {
      return !Double.isNaN(getJointSpaceKp());
   }

   default boolean hasJointSpaceKd()
   {
      return !Double.isNaN(getJointSpaceKd());
   }

   default boolean hasActuatorSpaceKp()
   {
      return !Double.isNaN(getActuatorSpaceKp());
   }

   default boolean hasActuatorSpaceKd()
   {
      return !Double.isNaN(getActuatorSpaceKd());
   }

   default boolean hasSpringSpaceKp()
   {
      return !Double.isNaN(getSpringSpaceKp());
   }

   default boolean hasSpringSpaceKd()
   {
      return !Double.isNaN(getSpringSpaceKd());
   }
}

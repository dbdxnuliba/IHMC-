package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains;

public class LegConfigurationGains implements LegConfigurationGainsBasics
{
   private double jointSpaceKp = Double.NaN;
   private double jointSpaceKd = Double.NaN;
   private double actuatorSpaceKp = Double.NaN;
   private double actuatorSpaceKd = Double.NaN;
   private double springSpaceKp = Double.NaN;
   private double springSpaceKd = Double.NaN;

   @Override
   public void setJointSpaceKp(double kp)
   {
      jointSpaceKp = kp;
   }

   @Override
   public void setJointSpaceKd(double kd)
   {
      jointSpaceKd = kd;
   }

   @Override
   public void setActuatorSpaceKp(double kp)
   {
      actuatorSpaceKp = kp;
   }

   @Override
   public void setActuatorSpaceKd(double kd)
   {
      actuatorSpaceKd = kd;
   }

   @Override
   public void setSpringSpaceKp(double kp)
   {
      springSpaceKp = kp;
   }

   @Override
   public void setSpringSpaceKd(double kd)
   {
      springSpaceKd = kd;
   }

   @Override
   public double getJointSpaceKp()
   {
      return jointSpaceKp;
   }

   @Override
   public double getJointSpaceKd()
   {
      return jointSpaceKd;
   }

   @Override
   public double getActuatorSpaceKp()
   {
      return actuatorSpaceKp;
   }

   @Override
   public double getActuatorSpaceKd()
   {
      return actuatorSpaceKd;
   }

   @Override
   public double getSpringSpaceKp()
   {
      return springSpaceKp;
   }

   @Override
   public double getSpringSpaceKd()
   {
      return springSpaceKd;
   }
}

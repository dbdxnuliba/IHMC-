package us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.gains;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoLegConfigurationGains implements LegConfigurationGainsBasics
{
   private final YoDouble jointSpaceKp;
   private final YoDouble jointSpaceKd;
   private final YoDouble actuatorSpaceKp;
   private final YoDouble actuatorSpaceKd;
   private final YoDouble springSpaceKp;
   private final YoDouble springSpaceKd;

   public YoLegConfigurationGains(String prefix, YoVariableRegistry registry)
   {
      jointSpaceKp = new YoDouble(prefix + "_JointSpaceKp", registry);
      jointSpaceKd = new YoDouble(prefix + "_JointSpaceKd", registry);
      actuatorSpaceKp = new YoDouble(prefix + "_ActuatorSpaceKp", registry);
      actuatorSpaceKd = new YoDouble(prefix + "_ActuatorSpaceKd", registry);
      springSpaceKp = new YoDouble(prefix + "_SpringSpaceKp", registry);
      springSpaceKd = new YoDouble(prefix + "_SpringSpaceKd", registry);

      setJointSpaceKp(Double.NaN);
      setJointSpaceKd(Double.NaN);
      setActuatorSpaceKp(Double.NaN);
      setActuatorSpaceKd(Double.NaN);
      setSpringSpaceKp(Double.NaN);
      setSpringSpaceKd(Double.NaN);
   }

   public YoLegConfigurationGains(String prefix, LegConfigurationGainsReadOnly gains, YoVariableRegistry registry)
   {
      jointSpaceKp = new YoDouble(prefix + "_JointSpaceKp", registry);
      jointSpaceKd = new YoDouble(prefix + "_JointSpaceKd", registry);
      actuatorSpaceKp = new YoDouble(prefix + "_ActuatorSpaceKp", registry);
      actuatorSpaceKd = new YoDouble(prefix + "_ActuatorSpaceKd", registry);
      springSpaceKp = new YoDouble(prefix + "_SpringSpaceKp", registry);
      springSpaceKd = new YoDouble(prefix + "_SpringSpaceKd", registry);

      set(gains);
   }

   @Override
   public void setJointSpaceKp(double kp)
   {
      jointSpaceKp.set(kp);
   }

   @Override
   public void setJointSpaceKd(double kd)
   {
      jointSpaceKd.set(kd);
   }

   @Override
   public void setActuatorSpaceKp(double kp)
   {
      actuatorSpaceKp.set(kp);
   }

   @Override
   public void setActuatorSpaceKd(double kd)
   {
      actuatorSpaceKd.set(kd);
   }

   @Override
   public void setSpringSpaceKp(double kp)
   {
      springSpaceKp.set(kp);
   }

   @Override
   public void setSpringSpaceKd(double kd)
   {
      springSpaceKd.set(kd);
   }

   @Override
   public double getJointSpaceKp()
   {
      return jointSpaceKp.getDoubleValue();
   }

   @Override
   public double getJointSpaceKd()
   {
      return jointSpaceKd.getDoubleValue();
   }

   @Override
   public double getActuatorSpaceKp()
   {
      return actuatorSpaceKp.getDoubleValue();
   }

   @Override
   public double getActuatorSpaceKd()
   {
      return actuatorSpaceKd.getDoubleValue();
   }

   @Override
   public double getSpringSpaceKp()
   {
      return springSpaceKp.getDoubleValue();
   }

   @Override
   public double getSpringSpaceKd()
   {
      return springSpaceKd.getDoubleValue();
   }
}

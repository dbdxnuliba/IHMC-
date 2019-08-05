package us.ihmc.robotEnvironmentAwareness.fusion.parameters;

public class PlanarRegionPropagationParameters
{
   private static final double DEFAULT_PROXIMITY_THRESHOLD = 0.02;
   private static final double DEFAULT_PLANARITY_THRESHOLD = Math.cos(Math.PI / 180 * 30);

   private static final boolean DEFAULT_ENABLE_EXTENDING = true;
   private static final boolean DEFAULT_UPDATE_EXTENDED_DATA = false;
   private static final double DEFAULT_EXTENDING_DISTANCE_THRESHOLD = 0.01;
   private static final double DEFAULT_EXTENDING_RADIUS_THRESHOLD = 0.03;

   private double proximityThreshold;
   private double planarityThreshold;
   private boolean enableExtending;
   private boolean updateExtendedData;
   private double extendingDistanceThreshold;
   private double extendingRadiusThreshold;

   public PlanarRegionPropagationParameters()
   {
      setDefaultParameters();
   }

   public PlanarRegionPropagationParameters(PlanarRegionPropagationParameters other)
   {
      set(other);
   }

   public void setDefaultParameters()
   {
      proximityThreshold = DEFAULT_PROXIMITY_THRESHOLD;
      planarityThreshold = DEFAULT_PLANARITY_THRESHOLD;

      enableExtending = DEFAULT_ENABLE_EXTENDING;
      updateExtendedData = DEFAULT_UPDATE_EXTENDED_DATA;
      extendingDistanceThreshold = DEFAULT_EXTENDING_DISTANCE_THRESHOLD;
      extendingRadiusThreshold = DEFAULT_EXTENDING_RADIUS_THRESHOLD;
   }

   public void set(PlanarRegionPropagationParameters other)
   {
      proximityThreshold = other.proximityThreshold;
      planarityThreshold = other.planarityThreshold;

      enableExtending = other.enableExtending;
      updateExtendedData = other.updateExtendedData;
      extendingDistanceThreshold = other.extendingDistanceThreshold;
      extendingRadiusThreshold = other.extendingRadiusThreshold;
   }

   public double getProximityThreshold()
   {
      return proximityThreshold;
   }

   public double getPlanarityThreshold()
   {
      return planarityThreshold;
   }

   public boolean isExtendingEnabled()
   {
      return enableExtending;
   }

   public boolean isUpdateExtendedData()
   {
      return updateExtendedData;
   }

   public double getExtendingDistanceThreshold()
   {
      return extendingDistanceThreshold;
   }

   public double getExtendingRadiusThreshold()
   {
      return extendingRadiusThreshold;
   }

   public void setProximityThreshold(double proximityThreshold)
   {
      this.proximityThreshold = proximityThreshold;
   }

   public void setPlanarityThreshold(double planarityThreshold)
   {
      this.planarityThreshold = planarityThreshold;
   }

   public void setPlanarityThresholdInDegree(double planarityThresholdInDegree)
   {
      this.planarityThreshold = Math.cos(Math.PI / 180 * planarityThresholdInDegree);
   }

   public void setEnableExtending(boolean enableExtending)
   {
      this.enableExtending = enableExtending;
   }

   public void setUpdateExtendedData(boolean updateExtendedData)
   {
      this.updateExtendedData = updateExtendedData;
   }

   public void setExtendingDistanceThreshold(double extendingDistanceThreshold)
   {
      this.extendingDistanceThreshold = extendingDistanceThreshold;
   }

   public void setExtendingRadiusThreshold(double extendingRadiusThreshold)
   {
      this.extendingRadiusThreshold = extendingRadiusThreshold;
   }
}

package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.tools.property.StoredPropertySet;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

public class BipedFootstepPlannerCostParameters implements BipedFootstepPlannerCostParametersReadOnly
{
   private final StoredPropertySet propertySet;

   public BipedFootstepPlannerCostParameters()
   {
      this(null);
   }

   public BipedFootstepPlannerCostParameters(BipedFootstepPlannerCostParametersReadOnly parameters)
   {
      this.propertySet = new StoredPropertySet(BipedFootstepPlannerCostParameterKeys.keys,
                                               getClass(),
                                               "ihmc-open-robotics-software",
                                               "ihmc-footstep-planning/src/main/resources");

      if (parameters != null)
         set(parameters);
   }

   public void set(BipedFootstepPlannerCostParametersReadOnly parameters)
   {
      propertySet.setAllValues(parameters.getStoredPropertySet().getAllValues());
   }

   public void setUseQuadraticDistanceCost(boolean useQuadraticDistanceCost)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.useQuadraticDistanceCost, useQuadraticDistanceCost);
   }

   public void setUseQuadraticHeightCost(boolean useQuadraticHeightCost)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.useQuadraticHeightCost, useQuadraticHeightCost);
   }

   public void setYawWeight(double yawWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.yawWeight, yawWeight);
   }

   public void setPitchWeight(double pitchWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.pitchWeight, pitchWeight);
   }

   public void setRollWeight(double rollWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.rollWeight, rollWeight);
   }

   public void setCostPerStep(double costPerStep)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.costPerStep, costPerStep);
   }

   public void setAStarHeuristicsWeight(double heuristicsWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.aStarHeuristicsWeight, heuristicsWeight);
   }

   public void setVisGraphWithAStarHeuristicsWeight(double heuristicsWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.visGraphWithAStarHeuristicsWeight, heuristicsWeight);
   }

   public void setDepthFirstHeuristicsWeight(double heuristicsWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.depthFirstHeuristicsWeight, heuristicsWeight);
   }

   public void setBodyPathBasedHeuristicsWeight(double heuristicsWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.bodyPathBasedHeuristicsWeight, heuristicsWeight);
   }

   public void setForwardWeight(double forwardWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.forwardWeight, forwardWeight);
   }

   public void setLateralWeight(double lateralWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.lateralWeight, lateralWeight);
   }

   public void setStepUpWeight(double stepUpWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.stepUpWeight, stepUpWeight);
   }

   public void setStepDownWeight(double stepDownWeight)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.stepDownWeight, stepDownWeight);
   }

   public void setBoundingBoxCost(double boundingBoxCost)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.boundingBoxCost, boundingBoxCost);
   }

   public void setMaximum2dDistanceFromBoundingBoxToPenalize(double maximum2dDistanceFromBoundingBoxToPenalize)
   {
      propertySet.setValue(BipedFootstepPlannerCostParameterKeys.maximum2dDistanceFromBoundingBoxToPenalize, maximum2dDistanceFromBoundingBoxToPenalize);
   }

   @Override
   public StoredPropertySetReadOnly getStoredPropertySet()
   {
      return propertySet;
   }
}

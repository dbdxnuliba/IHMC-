package us.ihmc.footstepPlanning.ui.components;

import javafx.beans.property.Property;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.graphSearch.parameters.*;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlannerCostParametersProperty extends ParametersProperty<BipedFootstepPlannerCostParameters>
{
   private BooleanField useQuadraticDistanceCost = new BooleanField(BipedFootstepPlannerCostParameters::useQuadraticDistanceCost, (p, v) -> p.setUseQuadraticDistanceCost(v));
   private BooleanField useQuadraticHeightCost = new BooleanField(BipedFootstepPlannerCostParameters::useQuadraticHeightCost, (p, v) -> p.setUseQuadraticHeightCost(v));
   private DoubleField yawWeight = new DoubleField(BipedFootstepPlannerCostParameters::getYawWeight, (p, v) -> p.setYawWeight(v));
   private DoubleField pitchWeight = new DoubleField(BipedFootstepPlannerCostParameters::getPitchWeight, (p, v) -> p.setPitchWeight(v));
   private DoubleField rollWeight = new DoubleField(BipedFootstepPlannerCostParameters::getRollWeight, (p, v) -> p.setRollWeight(v));
   private DoubleField forwardWeight = new DoubleField(BipedFootstepPlannerCostParameters::getForwardWeight, (p, v) -> p.setForwardWeight(v));
   private DoubleField lateralWeight = new DoubleField(BipedFootstepPlannerCostParameters::getLateralWeight, (p, v) -> p.setLateralWeight(v));
   private DoubleField stepUpWeight = new DoubleField(BipedFootstepPlannerCostParameters::getStepUpWeight, (p, v) -> p.setStepUpWeight(v));
   private DoubleField stepDownWeight = new DoubleField(BipedFootstepPlannerCostParameters::getStepDownWeight, (p, v) -> p.setStepDownWeight(v));
   private DoubleField costPerStep = new DoubleField(BipedFootstepPlannerCostParameters::getCostPerStep, (p, v) -> p.setCostPerStep(v));
   private DoubleField aStarHeuristicsWeight = new DoubleField(p -> p.getAStarHeuristicsWeightProvider().getValue(), (p, v) -> p.setAStarHeuristicsWeight(v));
   private DoubleField visGraphWithAStarHeuristicsWeight = new DoubleField(p -> p.getVisGraphWithAStarHeuristicsWeightProvider().getValue(), (p, v) -> p.setVisGraphWithAStarHeuristicsWeight(v));
   private DoubleField depthFirstHeuristicsWeight = new DoubleField(p -> p.getDepthFirstHeuristicsWeightProvider().getValue(), (p, v) -> p.setDepthFirstHeuristicsWeight(v));
   private DoubleField bodyPathBasedHeuristicsWeight = new DoubleField(p -> p.getBodyPathBasedHeuristicsWeightProvider().getValue(), (p, v) -> p.setBodyPathBasedHeuristicsWeight(v));

   public FootstepPlannerCostParametersProperty(Object bean, String name)
   {
      this(bean, name, new BipedFootstepPlannerCostParameters());
   }

   public FootstepPlannerCostParametersProperty(Object bean, String name, BipedFootstepPlannerCostParametersReadOnly footstepPlannerParameters)
   {
      super(bean, name, new BipedFootstepPlannerCostParameters(footstepPlannerParameters));
   }

   public void setPlannerParameters(BipedFootstepPlannerCostParametersReadOnly parameters)
   {
      setValue(new BipedFootstepPlannerCostParameters(parameters));
   }

   @Override
   protected BipedFootstepPlannerCostParameters getValueCopy(BipedFootstepPlannerCostParameters valueToCopy)
   {
      return new BipedFootstepPlannerCostParameters(valueToCopy);
   }
   
   public void bidirectionalBindYawWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, yawWeight);
   }

   public void bidirectionalBindPitchWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, pitchWeight);
   }

   public void bidirectionalBindRollWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, rollWeight);
   }

   public void bidirectionalBindForwardWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, forwardWeight);
   }

   public void bidirectionalBindLateralWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, lateralWeight);
   }

   public void bidirectionalBindStepUpWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepUpWeight);
   }

   public void bidirectionalBindStepDownWeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, stepDownWeight);
   }

   public void bidirectionalBindCostPerStep(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, costPerStep);
   }

   public void bidirectionalBindHeuristicsWeight(AtomicReference<FootstepPlannerType> plannerTypeReference, Property<? extends Number> property)
   {
      if (plannerTypeReference.get() == null)
         return;

      bindFieldBidirectionalToConditionalNumberProperty(() -> plannerTypeReference.get().equals(FootstepPlannerType.A_STAR), property, aStarHeuristicsWeight);
      bindFieldBidirectionalToConditionalNumberProperty(() -> plannerTypeReference.get().equals(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR), property, visGraphWithAStarHeuristicsWeight);
      bindFieldBidirectionalToConditionalNumberProperty(() -> plannerTypeReference.get().equals(FootstepPlannerType.PLANAR_REGION_BIPEDAL), property, depthFirstHeuristicsWeight);
      bindFieldBidirectionalToConditionalNumberProperty(() -> plannerTypeReference.get().equals(FootstepPlannerType.SIMPLE_BODY_PATH), property, bodyPathBasedHeuristicsWeight);
   }

   public void bidirectionalBindUseQuadraticHeightCost(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, useQuadraticHeightCost);
   }

   public void bidirectionalBindUseQuadraticDistanceCost(Property<Boolean> property)
   {
      bindFieldBidirectionalToBooleanProperty(property, useQuadraticDistanceCost);
   }
}

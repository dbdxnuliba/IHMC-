package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SuperPixelNormalEstimationParameters;

public class SuperPixelNormalEstimationParametersProperty extends ParametersProperty<SuperPixelNormalEstimationParameters>
{
   private final DoubleField maxDistanceFromPlane = new DoubleField(SuperPixelNormalEstimationParameters::getMaxDistanceFromPlane, SuperPixelNormalEstimationParameters::setMaxDistanceFromPlane);
   private final DoubleField minConsensusRatio = new DoubleField(SuperPixelNormalEstimationParameters::getMinConsensusRatio, SuperPixelNormalEstimationParameters::setMinConsensusRatio);
   private final DoubleField maxAverageDeviationRatio = new DoubleField(SuperPixelNormalEstimationParameters::getMaxAverageDeviationRatio, SuperPixelNormalEstimationParameters::setMaxAverageDeviationRatio);
   private final IntegerField numberOfIterations = new IntegerField(SuperPixelNormalEstimationParameters::getNumberOfIterations, SuperPixelNormalEstimationParameters::setNumberOfIterations);
   private final BooleanField enableLeastSquaresEstimation = new BooleanField(SuperPixelNormalEstimationParameters::isLeastSquaresEstimationEnabled, SuperPixelNormalEstimationParameters::enableLeastSquaresEstimation);
   private final BooleanField updateUsingPca = new BooleanField(SuperPixelNormalEstimationParameters::updateUsingPCA, SuperPixelNormalEstimationParameters::updateUsingPCA);
   private final DoubleField minDistanceForNormalGuess = new DoubleField(SuperPixelNormalEstimationParameters::getMinDistanceForNormalGuess, SuperPixelNormalEstimationParameters::setMinDistanceForNormalGuess);
   private final IntegerField maxAttemptsForNormalGuess = new IntegerField(SuperPixelNormalEstimationParameters::getMaxAttemptsForNormalGuess, SuperPixelNormalEstimationParameters::setMaxAttemptsForNormalGuess);

   public SuperPixelNormalEstimationParametersProperty(Object bean, String name)
   {
      super(bean, name, new SuperPixelNormalEstimationParameters());
   }

   public void bindBidirectionalMaxDistanceFromPlane(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxDistanceFromPlane);
   }

   public void bindBidirectionalMinConsensusRatio(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minConsensusRatio);
   }

   public void bindBidirectionalMaxAverageDeviationRatio(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxAverageDeviationRatio);
   }

   public void bindBidirectionalNumberOfIterations(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, numberOfIterations);
   }

   public void bindBidirectionalEnableLeastSquaresEstimation(BooleanProperty property)
   {
      bindFieldBidirectionalToBooleanProperty(property, enableLeastSquaresEstimation);
   }

   public void bindBidirectionalMinDistanceForNormalGuess(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minDistanceForNormalGuess);
   }

   public void bindBidirectionalMaxAttemptsForNormalGuess(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxAttemptsForNormalGuess);
   }


   public void bindBidirectionalUpdateUsingPCA(BooleanProperty property)
   {
      bindFieldBidirectionalToBooleanProperty(property, updateUsingPca);
   }

   @Override
   protected SuperPixelNormalEstimationParameters getValueCopy(SuperPixelNormalEstimationParameters valueToCopy)
   {
      return new SuperPixelNormalEstimationParameters(valueToCopy);
   }
}

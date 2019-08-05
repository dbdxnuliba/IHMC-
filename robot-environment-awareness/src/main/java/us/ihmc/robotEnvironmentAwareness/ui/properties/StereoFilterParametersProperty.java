package us.ihmc.robotEnvironmentAwareness.ui.properties;

import javafx.beans.property.Property;
import us.ihmc.robotEnvironmentAwareness.planarRegion.StereoFilterParameters;

public class StereoFilterParametersProperty extends ParametersProperty<StereoFilterParameters>
{
   private final IntegerField numberOfBuffer = new IntegerField(StereoFilterParameters::getNumberOfBuffer, StereoFilterParameters::setNumberOfBuffer);
   private final IntegerField sizeOfBuffer = new IntegerField(StereoFilterParameters::getSizeOfBuffer, StereoFilterParameters::setSizeOfBuffer);
   private final DoubleField surfaceNormalUpperBound = new DoubleField(StereoFilterParameters::getSurfaceNormalUpperBoundDegree,
                                                                       StereoFilterParameters::setSurfaceNormalUpperBoundDegree);
   private final DoubleField surfaceNormalLowerBound = new DoubleField(StereoFilterParameters::getSurfaceNormalLowerBoundDegree,
                                                                       StereoFilterParameters::setSurfaceNormalLowerBoundDegree);

   public StereoFilterParametersProperty(Object bean, String name)
   {
      super(bean, name, new StereoFilterParameters());
   }

   @Override
   protected StereoFilterParameters getValueCopy(StereoFilterParameters valueToCopy)
   {
      return new StereoFilterParameters(valueToCopy);
   }

   public void bindBidirectionalNumberOfBuffer(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, numberOfBuffer);
   }

   public void bindBidirectionalSizeOfBuffer(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, sizeOfBuffer);
   }

   public void bindBidirectionalSurfaceNormalUpperBound(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, surfaceNormalUpperBound);
   }

   public void bindBidirectionalSurfaceNormalLowerBound(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, surfaceNormalLowerBound);
   }
}

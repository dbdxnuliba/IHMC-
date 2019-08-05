package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ImageSegmentationParametersProperty;
import us.ihmc.robotEnvironmentAwareness.ui.properties.PlanarRegionPropagationParametersProperty;
import us.ihmc.robotEnvironmentAwareness.ui.properties.SegmentationRawDataFilteringParametersProperty;
import us.ihmc.robotEnvironmentAwareness.ui.properties.SuperPixelNormalEstimationParametersProperty;

public class StereoREAAnchorPaneController
{
   private JavaFXMessager messager;

   @FXML private Label computationTime;
   @FXML private Label dataFilteringTime;
   @FXML private Label dataFusingTime;
   @FXML private Label segmentationTime;
   @FXML private ToggleButton enableREA;
   @FXML private Button clearREA;
   @FXML private Slider stereoBuffer;
   @FXML private Button showSegmentationRawData;
   @FXML private Slider superpixelSize;
   @FXML private Slider superpixelRuler;
   @FXML private Slider superpixelIterate;
   @FXML private ToggleButton enableConnectivity;
   @FXML private ToggleButton groupViaColors;
   @FXML private ToggleButton showRawData;
   @FXML private ToggleButton showFusedData;
   @FXML private ToggleButton showPlanarRegions;
   @FXML private ToggleButton runSingleThreaded;
   @FXML private Slider superpixelMinimumElement;
   @FXML private Slider minSparse;
   @FXML private Slider sparseRatio;
   @FXML private ToggleButton flyingPoint;
   @FXML private Slider minimumNeighbors;
   @FXML private Slider flyingPointDistance;
   @FXML private ToggleButton centrality;
   @FXML private Slider centralityRadius;
   @FXML private Slider centralityThreshold;
   @FXML private ToggleButton ellipticity;
   @FXML private Slider ellipticityLength;
   @FXML private Slider ellipticityThreshold;
   @FXML private Slider proximityThreshold;
   @FXML private Slider planarityThreshold;
   @FXML private CheckBox enableExtending;
   @FXML private CheckBox updateExtendedData;
   @FXML private Slider extendingDistanceThreshold;
   @FXML private Slider ExtendingRadius;

   @FXML private Slider maxNormalVariance;
   @FXML private Slider minNormalConsensus;

   @FXML private Slider rawMaxDistanceFromPlane;
   @FXML private Slider rawMaxDeviation;
   @FXML private Slider rawMinConsensus;
   @FXML private Slider rawRansacIterations;
   @FXML private CheckBox rawUsePCA;
   @FXML private CheckBox rawUseLeastSquares;
   @FXML private Slider rawMinDistanceForNormalGuess;
   @FXML private Slider rawMaxAttemptsForNormalGuess;



   private final ImageSegmentationParametersProperty imageSegmentationParametersProperty = new ImageSegmentationParametersProperty(this,
                                                                                                                                   "imageSegmentationParametersProperty");
   private final PlanarRegionPropagationParametersProperty planarRegionPropagationParametersProperty = new PlanarRegionPropagationParametersProperty(this,
                                                                                                                                                     "planarRegionPropagationParametersProperty");

   private final SegmentationRawDataFilteringParametersProperty segmentationRawDataFilteringParametersProperty = new SegmentationRawDataFilteringParametersProperty(this,
                                                                                                                                                                    "segmentationRawDataFilteringParametersProperty");
   private final SuperPixelNormalEstimationParametersProperty rawSuperPixelNormalEstimationParametersProperty = new SuperPixelNormalEstimationParametersProperty(this, "rawSuperPixelNormalEstimationParametersProperty");

   @FXML
   private Button runSREA;

   public void initialize(JavaFXMessager messager)
   {
      this.messager = messager;

      messager.bindBidirectional(LidarImageFusionAPI.EnableREA, enableREA.selectedProperty(), false);
      messager.bindBidirectional(LidarImageFusionAPI.ComputationTime, computationTime.textProperty(), false);
      messager.bindBidirectional(LidarImageFusionAPI.DataFilteringTime, dataFilteringTime.textProperty(), false);
      messager.bindBidirectional(LidarImageFusionAPI.DataFusingTime, dataFusingTime.textProperty(), false);
      messager.bindBidirectional(LidarImageFusionAPI.ImageSegmentationTime, segmentationTime.textProperty(), false);
      messager.bindBidirectional(LidarImageFusionAPI.StereoBufferSize, stereoBuffer.valueProperty(), numberToIntegerConverter, true);

      imageSegmentationParametersProperty.bindBidirectionalPixelSize(superpixelSize.valueProperty());
      imageSegmentationParametersProperty.bindBidirectionalPixelRuler(superpixelRuler.valueProperty());
      imageSegmentationParametersProperty.bindBidirectionalIterate(superpixelIterate.valueProperty());
      imageSegmentationParametersProperty.bindBidirectionalEnableConnectivity(enableConnectivity.selectedProperty());
      imageSegmentationParametersProperty.bindBidirectionalGroupViaColors(groupViaColors.selectedProperty());
      imageSegmentationParametersProperty.bindBidirectionalMinElementSize(superpixelMinimumElement.valueProperty());
      messager.bindBidirectional(LidarImageFusionAPI.ImageSegmentationParameters, imageSegmentationParametersProperty, true);

      segmentationRawDataFilteringParametersProperty.bindBidirectionalSparseThreshold(minSparse.valueProperty(), sparseRatio.valueProperty());
      segmentationRawDataFilteringParametersProperty.bindBidirectionalEnableFilterFlyingPoint(flyingPoint.selectedProperty());
      segmentationRawDataFilteringParametersProperty.bindBidirectionalFlyingPointParameters(flyingPointDistance.valueProperty(),
                                                                                            minimumNeighbors.valueProperty());
      segmentationRawDataFilteringParametersProperty.bindBidirectionalEnableFilterCentrality(centrality.selectedProperty());
      segmentationRawDataFilteringParametersProperty.bindBidirectionalCentralityParameters(centralityRadius.valueProperty(),
                                                                                           centralityThreshold.valueProperty());
      segmentationRawDataFilteringParametersProperty.bindBidirectionalEnableFilterEllipticity(ellipticity.selectedProperty());
      segmentationRawDataFilteringParametersProperty.bindBidirectionalEllipticityParameters(ellipticityLength.valueProperty(),
                                                                                            ellipticityThreshold.valueProperty());
//      segmentationRawDataFilteringParametersProperty.bindBidirectionalMaxNormalVariance(maxNormalVariance.valueProperty());
//      segmentationRawDataFilteringParametersProperty.bindBidirectionalMinNormalConsensus(minNormalConsensus.valueProperty());
      messager.bindBidirectional(LidarImageFusionAPI.SegmentationRawDataFilteringParameters, segmentationRawDataFilteringParametersProperty, true);

      planarRegionPropagationParametersProperty.bindBidirectionalProximityThreshold(proximityThreshold.valueProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalPlanarityThreshold(planarityThreshold.valueProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalEnableExtending(enableExtending.selectedProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalUpdateExtendedData(updateExtendedData.selectedProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalExtendingDistanceThreshold(extendingDistanceThreshold.valueProperty());
      planarRegionPropagationParametersProperty.bindBidirectionalExtendingRadiusThreshold(ExtendingRadius.valueProperty());
      messager.bindBidirectional(LidarImageFusionAPI.PlanarRegionPropagationParameters, planarRegionPropagationParametersProperty, true);


      rawSuperPixelNormalEstimationParametersProperty.bindBidirectionalMaxDistanceFromPlane(rawMaxDistanceFromPlane.valueProperty());
      rawSuperPixelNormalEstimationParametersProperty.bindBidirectionalMaxAverageDeviationRatio(rawMaxDeviation.valueProperty());
      rawSuperPixelNormalEstimationParametersProperty.bindBidirectionalMinConsensusRatio(rawMinConsensus.valueProperty());
      rawSuperPixelNormalEstimationParametersProperty.bindBidirectionalNumberOfIterations(rawRansacIterations.valueProperty());
      rawSuperPixelNormalEstimationParametersProperty.bindBidirectionalUpdateUsingPCA(rawUsePCA.selectedProperty());
      rawSuperPixelNormalEstimationParametersProperty.bindBidirectionalEnableLeastSquaresEstimation(rawUseLeastSquares.selectedProperty());
      rawSuperPixelNormalEstimationParametersProperty.bindBidirectionalMinDistanceForNormalGuess(rawMinDistanceForNormalGuess.valueProperty());
      rawSuperPixelNormalEstimationParametersProperty.bindBidirectionalMaxAttemptsForNormalGuess(rawMaxAttemptsForNormalGuess.valueProperty());
      messager.bindBidirectional(LidarImageFusionAPI.SuperPixelNormalEstimationParameters, rawSuperPixelNormalEstimationParametersProperty, true);
   }

   private final PropertyToMessageTypeConverter<Integer, Number> numberToIntegerConverter = new PropertyToMessageTypeConverter<Integer, Number>()
   {
      @Override
      public Integer convert(Number propertyValue)
      {
         return propertyValue.intValue();
      }

      @Override
      public Number interpret(Integer newValue)
      {
         return new Double(newValue.intValue());
      }
   };

   public void clearREA()
   {
      messager.submitMessage(LidarImageFusionAPI.ClearREA, true);
   }

   public void showRawData()
   {
      messager.submitMessage(LidarImageFusionAPI.ShowRawSuperPixelData, showRawData.isSelected());
   }

   public void showFusedData()
   {
      messager.submitMessage(LidarImageFusionAPI.ShowFusedSuperPixelData, showFusedData.isSelected());
   }

   public void showPlanarRegions()
   {
      messager.submitMessage(LidarImageFusionAPI.ShowPlanarRegions, showPlanarRegions.isSelected());
   }

   public void runSingleThreaded()
   {
      messager.submitMessage(LidarImageFusionAPI.RunSingleThreaded, runSingleThreaded.isSelected());
   }

   public void showProjection()
   {
      messager.submitMessage(LidarImageFusionAPI.ShowStereoBufferProjection, true);
   }

   public void runSREA()
   {
      messager.submitMessage(LidarImageFusionAPI.RunStereoREA, true);
   }
}

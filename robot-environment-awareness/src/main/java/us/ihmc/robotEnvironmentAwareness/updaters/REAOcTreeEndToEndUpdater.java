package us.ihmc.robotEnvironmentAwareness.updaters;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.publisherTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberTopicNameGenerator;

import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.BoundingBoxMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomPlanarRegionHandler;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.StereoFilterParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

public class REAOcTreeEndToEndUpdater
{
   private static final double OCTREE_RESOLUTION = 0.02;
   private static final double insertingMinimumRange = 0.5;
   private static final double insertingMaximumRange = 5.0;

   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionPointCloudMessage = new AtomicReference<>(null);

   private final NormalOcTree mainOctree = new NormalOcTree(OCTREE_RESOLUTION);

   private final PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();

   private final Messager reaMessager;

   private final AtomicReference<BoundingBoxParametersMessage> atomicBoundingBoxParameters;
   private final AtomicReference<NormalEstimationParameters> normalEstimationParameters;
   private final AtomicReference<StereoFilterParameters> stereoFilterParameters;

   private final AtomicReference<PlanarRegionSegmentationParameters> planarRegionSegmentationParameters;
   private final AtomicReference<ConcaveHullFactoryParameters> concaveHullFactoryParameters;
   private final AtomicReference<PolygonizerParameters> polygonizerParameters;

   public REAOcTreeEndToEndUpdater(Messager reaMessager, Ros2Node ros2Node)
   {
      this.reaMessager = reaMessager;

      mainOctree.enableParallelComputationForNormals(true);
      mainOctree.enableParallelInsertionOfMisses(true);

      reaMessager.registerTopicListener(REAModuleAPI.StereoREAEndToEndRun, (content) -> run());

      atomicBoundingBoxParameters = reaMessager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters,
                                                            BoundingBoxMessageConverter.createBoundingBoxParametersMessage(0.0f, -2.0f, -3.0f, 5.0f, 2.0f,
                                                                                                                           1.5f));
      normalEstimationParameters = reaMessager.createInput(REAModuleAPI.NormalEstimationParameters, new NormalEstimationParameters());
      stereoFilterParameters = reaMessager.createInput(REAModuleAPI.StereoFilterParameters, new StereoFilterParameters());

      planarRegionSegmentationParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsSegmentationParameters, new PlanarRegionSegmentationParameters());
      concaveHullFactoryParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsConcaveHullParameters, new ConcaveHullFactoryParameters());
      polygonizerParameters = reaMessager.createInput(REAModuleAPI.PlanarRegionsPolygonizerParameters, new PolygonizerParameters());
   }

   public void handleStereoPointCloudMessage(StereoVisionPointCloudMessage stereoVisionPointCloudMessage)
   {
      latestStereoVisionPointCloudMessage.set(stereoVisionPointCloudMessage);
   }

   private NormalOcTree bufferOctree = new NormalOcTree(OCTREE_RESOLUTION);

   private void run()
   {
      System.out.println("run ");
      long startTime = System.nanoTime();
      bufferOctree.clear();

      //int numberOfBuffer = stereoFilterParameters.get().getNumberOfBuffer();   // TODO: use executor service
      int numberOfBuffer = 1;
      for (int i = 0; i < numberOfBuffer; i++)
         updateBuffer();
      
      System.out.println("update " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
      
      calculate();
      publish();

      System.out.println("end to end time " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));
   }

   private void updateBuffer()
   {
      System.out.println("updateBuffer");
      StereoVisionPointCloudMessage stereoMessage = latestStereoVisionPointCloudMessage.getAndSet(null);

      if (stereoMessage != null)
      {
         ScanCollection newScan = new ScanCollection();
         newScan.setSubSampleSize(stereoFilterParameters.get().getSizeOfBuffer());
         newScan.addScan(stereoMessage.getPointCloud().toArray(), stereoMessage.getSensorPosition());
         bufferOctree.insertScanCollection(newScan, false);
         System.out.println(bufferOctree.getNumberOfLeafNodes() + " " + newScan.getNumberOfPoints() + " " + stereoFilterParameters.get().getSizeOfBuffer());
      }
   }

   private void calculate()
   {
      long startTime = System.nanoTime();
      System.out.println("calculate");

      OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw();

      Point3D min = atomicBoundingBoxParameters.get().getMin();
      Point3D max = atomicBoundingBoxParameters.get().getMax();

      boundingBox.setLocalMinMaxCoordinates(min, max);
      boundingBox.update(mainOctree.getResolution(), mainOctree.getTreeDepth());

      System.out.println("mainOctree.getTreeDepth() " + mainOctree.getTreeDepth());

      mainOctree.clearNormals();
      mainOctree.setBoundingBox(boundingBox);
      mainOctree.setBoundsInsertRange(insertingMinimumRange, insertingMaximumRange);
      mainOctree.setNormalEstimationParameters(normalEstimationParameters.get());

      if (bufferOctree != null)
      {
         PointCloud pointCloud = new PointCloud();
         bufferOctree.forEach(node -> pointCloud.add(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ()));
         Scan scan = new Scan(new Point3D(), pointCloud);
         Set<NormalOcTreeNode> updatedNodes = new HashSet<>();
         mainOctree.insertScan(scan, updatedNodes, null);
      }

      mainOctree.updateNormals();
      
      System.out.println("updateNormals " + Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));

      segmentationCalculator.clear();
      segmentationCalculator.removeDeadNodes();

      segmentationCalculator.setBoundingBox(mainOctree.getBoundingBox());
      segmentationCalculator.setParameters(planarRegionSegmentationParameters.get());
      segmentationCalculator.setStereoFilterParameters(stereoFilterParameters.get());

      segmentationCalculator.compute(mainOctree.getRoot());

   }

   private void publish()
   {
      //TODO: add also publish with REAPlanarRegionPublicNetworkProvider
      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();
      System.out.println("rawData.size() " + rawData.size());

      ConcaveHullFactoryParameters concaveHullFactoryParameters = this.concaveHullFactoryParameters.get();
      PolygonizerParameters polygonizerParameters = this.polygonizerParameters.get();

      PlanarRegionsList planarRegionsList = PlanarRegionPolygonizer.createPlanarRegionsList(rawData, concaveHullFactoryParameters, polygonizerParameters);

      System.out.println("submitMessage");
      if (planarRegionsList != null)
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
   }
}

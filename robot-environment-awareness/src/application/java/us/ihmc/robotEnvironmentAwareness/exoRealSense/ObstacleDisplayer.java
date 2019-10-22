package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.publisherTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberTopicNameGenerator;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeBuffer;
import us.ihmc.robotEnvironmentAwareness.updaters.REAOcTreeUpdater;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionFeatureUpdater;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

/*
 * main class that connect realsense D415 (RosNodeWithD415BridgeToRos2), runs point cloud through planar region finder (todo JOBY), evaluate obstacles (obstacleDistance) and present them to hololense
 */
public class ObstacleDisplayer
{   
   //variables
   private static Ros2Node ros2Node;

   private static final int THREAD_PERIOD_MILLISECONDS = 200;
   private static final int BUFFER_THREAD_PERIOD_MILLISECONDS = 10;
   private static final double DEFAULT_OCTREE_RESOLUTION = 0.02;

   protected static final boolean DEBUG = true;
   
   private final REAOcTreeBuffer stereoVisionBufferUpdater;
   private final REAOcTreeUpdater mainUpdater; 
   private final REAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final REAPlanarRegionPublicNetworkProvider planarRegionNetworkProvider;

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(2, getClass(), ExceptionHandling.CATCH_AND_REPORT);
   private ScheduledFuture<?> scheduled;
   private final Messager reaMessager;  
   
   private UDPDataSender sender;
   
   //functions
   public static void main(String[] args)
   {
      try {
         ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, ROS2Tools.REA.getNodeName());
         new RealSenseBridgeRos2(ros2Node); //connection to realsense D415         

         ObstacleDisplayer module = ObstacleDisplayer.createIntraprocessModule();
         module.start();         
      }
      catch (Exception ex) {
         ex.printStackTrace();         
      }
   }
      
   private ObstacleDisplayer(Messager reaMessager, File configurationFile) throws IOException
   {
      this.reaMessager = reaMessager;
                                                   
      stereoVisionBufferUpdater = new REAOcTreeBuffer(DEFAULT_OCTREE_RESOLUTION, reaMessager, REAModuleAPI.StereoVisionBufferEnable, false,
                                                      REAModuleAPI.StereoVisionBufferOcTreeCapacity, 1000000, REAModuleAPI.StereoVisionBufferMessageCapacity, 1,
                                                      REAModuleAPI.RequestStereoVisionBuffer, REAModuleAPI.StereoVisionBufferState);
      REAOcTreeBuffer[] bufferUpdaters = new REAOcTreeBuffer[] { stereoVisionBufferUpdater};
      mainUpdater = new REAOcTreeUpdater(DEFAULT_OCTREE_RESOLUTION, bufferUpdaters, reaMessager);
      planarRegionFeatureUpdater = new REAPlanarRegionFeatureUpdater(reaMessager);

      ROS2Tools.createCallbackSubscription(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator(),
                                           this::dispatchStereoVisionPointCloudMessage);

      FilePropertyHelper filePropertyHelper = new FilePropertyHelper(configurationFile);
      loadConfigurationFile(filePropertyHelper);

      reaMessager.registerTopicListener(REAModuleAPI.SaveBufferConfiguration, (content) -> stereoVisionBufferUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveMainUpdaterConfiguration, (content) -> mainUpdater.saveConfiguration(filePropertyHelper));
      reaMessager.registerTopicListener(REAModuleAPI.SaveRegionUpdaterConfiguration,
                                        (content) -> planarRegionFeatureUpdater.saveConfiguration(filePropertyHelper));

      planarRegionNetworkProvider = new REAPlanarRegionPublicNetworkProvider(reaMessager, planarRegionFeatureUpdater, ros2Node, publisherTopicNameGenerator,
                                                                             subscriberTopicNameGenerator);

      // At the very end, we force the modules to submit their state so duplicate inputs have consistent values.
      reaMessager.submitMessage(REAModuleAPI.RequestEntireModuleState, true); 
      
      reaMessager.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.StereoVisionBufferEnable, true);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, false);
      reaMessager.submitMessage(REAModuleAPI.UIOcTreeDisplayType, DisplayType.HIDE);
      
      sender = new UDPDataSender();
   }

   private void dispatchStereoVisionPointCloudMessage(Subscriber<StereoVisionPointCloudMessage> subscriber)
   {
      StereoVisionPointCloudMessage message = subscriber.takeNextData();
      stereoVisionBufferUpdater.handleStereoVisionPointCloudMessage(message);      
      mainUpdater.handleStereoVisionPointCloudMessage(message);
   }

   private void loadConfigurationFile(FilePropertyHelper filePropertyHelper)
   {
      stereoVisionBufferUpdater.loadConfiguration(filePropertyHelper);
      mainUpdater.loadConfiguration(filePropertyHelper);
      planarRegionFeatureUpdater.loadConfiguration(filePropertyHelper);
   }

   private void mainUpdate()
   {
      if (isThreadInterrupted())
         return;

      boolean ocTreeUpdateSuccess = true;

      try
      {
         NormalOcTree mainOctree = mainUpdater.getMainOctree();
         
         mainUpdater.clearOcTree();
         mainUpdater.update();

         if (isThreadInterrupted())
            return;

         planarRegionFeatureUpdater.update(mainOctree);
         
         double distance = obstacleDistance(planarRegionFeatureUpdater.getPlanarRegionsList());
         sender.sendDistance(distance);

         planarRegionNetworkProvider.update(ocTreeUpdateSuccess);
         planarRegionNetworkProvider.publishCurrentState();
      }
      catch (Exception e)
      {
         if (DEBUG)
         {
            e.printStackTrace();
         }
         else
         {
            LogTools.error(e.getClass().getSimpleName());
         }
      }
   }

   /*
    * returns -1 if there is no obstacle otherwise return distance to obstacle
    */
   private double obstacleDistance(PlanarRegionsList planarRegionsList) {
      
      //params
      double distance = -1.0;            
      double positiveAngle = 135;
      double positiveD2Distance = 0.1;
      LinkedList<Double> distanceList = new LinkedList<Double>();
      
      for(int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++) {
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(i);
         Vector3D vector = planarRegion.getNormal();
         //System.out.println(vector.toString());
         double angle = Math.acos(vector.getZ())*180/Math.PI;
         //System.out.println(angle + ", " + (angle > positiveAngle ? "true" : "false"));
         if(angle > positiveAngle) {
            double D2Distance = planarRegion.distanceToPointByProjectionOntoXYPlane(0.0, 0.0);
            //System.out.println(D2Distance);
            if(D2Distance < positiveD2Distance) {
               distance = planarRegion.getPlaneZGivenXY(0, 0);
               //System.out.println("obstacle at " + distance + " meters");
               distanceList.add(distance);                        
            }
         }               
      }
      
      //System.out.println("-----------------------------------");
      for(int i = 0; i < distanceList.size() -1; i++){
         if(distanceList.get(i) < distance) {
            distance = distanceList.get(i);
         }
      }
      return distance;   
   }

   private boolean isThreadInterrupted()
   {
      return Thread.interrupted() || scheduled == null || scheduled.isCancelled();
   }

   public void start() throws IOException
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this::mainUpdate, 0, THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
         executorService.scheduleAtFixedRate(stereoVisionBufferUpdater.createBufferThread(), 0, BUFFER_THREAD_PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop() throws Exception
   {
      reaMessager.closeMessager();
      ros2Node.destroy();

      if (scheduled != null)
      {
         scheduled.cancel(true);
         scheduled = null;
      }

      if (executorService != null)
      {
         executorService.shutdownNow();
         executorService = null;
      }
   }

   public static ObstacleDisplayer createIntraprocessModule() throws Exception
   {
      String configurationFilePath = "./Configurations/defaultREAModuleConfiguration.txt";
      KryoMessager messager = KryoMessager.createIntraprocess(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                              REACommunicationProperties.getPrivateNetClassList());
      messager.setAllowSelfSubmit(true);
      messager.startMessager();

      File configurationFile = new File(configurationFilePath);
      try
      {
         configurationFile.getParentFile().mkdirs();
         configurationFile.createNewFile();
      }
      catch (IOException e)
      {
         System.out.println(configurationFile.getAbsolutePath());
         e.printStackTrace();
      }

      return new ObstacleDisplayer(messager, configurationFile);
   }
}
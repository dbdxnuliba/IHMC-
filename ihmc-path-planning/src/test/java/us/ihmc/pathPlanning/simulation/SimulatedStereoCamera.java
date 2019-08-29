package us.ihmc.pathPlanning.simulation;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXVisualizers.IdMappedColorFunction;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.hardware.MultisenseInformation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.Ray3d;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.awt.*;

public class SimulatedStereoCamera
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final MovingReferenceFrame cameraSensorFrame;
   private final IntrinsicParameters intrinsicParameters;

   private PlanarRegionsList planarRegionsList;

   private final double widthFieldOfView;
   private final double heightFieldOfView;

   private final DoubleProvider timeProvider;

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;

   private final Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "stereoVisionPublisherNode");



   /**
    * The camera sensor frame is defined with a positive x forward and z up, as if you were looking at the picture.
    */
   public SimulatedStereoCamera(DoubleProvider timeProvider, MovingReferenceFrame cameraSensorFrame, IntrinsicParameters intrinsicParameters, double widthFieldOfView, double heightFieldOfView)
   {
      this.timeProvider = timeProvider;
      this.cameraSensorFrame = cameraSensorFrame;
      this.intrinsicParameters = intrinsicParameters;
      this.widthFieldOfView = widthFieldOfView;
      this.heightFieldOfView = heightFieldOfView;

      // FIX THIS
      RosMainNode rosMainNode = new RosMainNode(masterURI, "StereoVisionPublisher", true);
      rosMainNode.attachSubscriber(MultisenseInformation.getStereoVisionPointCloudTopicName(), this);

      rosMainNode.execute();

      stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());

   }

   public void setPlanarRegionsEnvironment(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   private void update()
   {
      int numberWide = (int) intrinsicParameters.getFx();
      int numberHigh = (int) intrinsicParameters.getFy();
      double widthIncrement = widthFieldOfView / numberWide;
      double heightIncrement = heightFieldOfView / numberHigh;

      FrameVector3D rayDirection = new FrameVector3D(cameraSensorFrame);
      FramePoint3D rayOrigin = new FramePoint3D(cameraSensorFrame);
      rayOrigin.changeFrame(worldFrame);

      int numberOfPoints = numberHigh * numberWide;
      Point3DReadOnly[] pointCloud = new Point3DReadOnly[numberOfPoints];
      Color[] colors = new Color[numberOfPoints];

      for (int widthIdx = 0; widthIdx < numberWide; widthIdx++)
      {
         for (int heightIdx = 0; heightIdx < numberHigh; heightIdx++)
         {
            double widthAngle = 0.5 * widthFieldOfView - widthIdx * widthIncrement;
            double heightAngle = 0.5 * heightFieldOfView - heightIdx * heightIncrement;
            rayDirection.setToZero(cameraSensorFrame);
            rayDirection.set(Math.cos(widthAngle) * Math.cos(heightAngle), Math.cos(heightAngle) * Math.sin(widthAngle), Math.sin(heightAngle));
            rayDirection.changeFrame(worldFrame);

            FramePoint3D intersectionWidthWorld = new FramePoint3D(worldFrame, PlanarRegionTools.intersectRegionsWithRay(planarRegionsList, rayOrigin, rayDirection));
            intersectionWidthWorld.changeFrame(cameraSensorFrame);

            pointCloud[widthIdx * numberWide + heightIdx] = intersectionWidthWorld;
            colors[widthIdx * numberWide + heightIdx] = IdMappedColorFunction.INSTANCE.apply(regionId);
         }
      }

      long timestamp = Conversions.secondsToNanoseconds(timeProvider.getValue());
      float[] pointCloudBuffer = new float[3 * numberOfPoints];
      int[] colorsInteger = new int[numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3DReadOnly scanPoint = pointCloud[i];

         pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
         pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
         pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();

         colorsInteger[i] = colors[i].getRGB();
      }

      StereoVisionPointCloudMessage stereoVisionMessage = MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer, colorsInteger);

      stereoVisionPublisher.publish(stereoVisionMessage);
   }
}

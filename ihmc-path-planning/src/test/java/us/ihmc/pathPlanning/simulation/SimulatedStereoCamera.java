package us.ihmc.pathPlanning.simulation;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
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
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.awt.*;
import java.util.concurrent.*;

public class SimulatedStereoCamera implements Runnable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final MovingReferenceFrame cameraSensorFrame;
   private final IntrinsicParameters intrinsicParameters;

   private PlanarRegionsList planarRegionsList;

   private final double widthFieldOfView;
   private final double heightFieldOfView;

   private final DoubleProvider timeProvider;

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;

   private boolean start = false;
   private final ScheduledExecutorService executorService = Executors.newScheduledThreadPool(1, ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private final ScheduledFuture<?> scheduledFuture;

   /**
    * The camera sensor frame is defined with a positive x forward and z up, as if you were looking at the picture.
    */
   public SimulatedStereoCamera(DoubleProvider timeProvider, Ros2Node ros2Node, MovingReferenceFrame cameraSensorFrame, IntrinsicParameters intrinsicParameters,
                                double widthFieldOfView, double heightFieldOfView, double updateFrequencyHz)
   {
      this.timeProvider = timeProvider;
      this.cameraSensorFrame = cameraSensorFrame;
      this.intrinsicParameters = intrinsicParameters;
      this.widthFieldOfView = widthFieldOfView;
      this.heightFieldOfView = heightFieldOfView;

      stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());

      scheduledFuture = executorService.scheduleAtFixedRate(this, 0, (long) (1.0 / updateFrequencyHz), TimeUnit.SECONDS);
   }

   public void setPlanarRegionsEnvironment(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void start()
   {
      start = true;
   }
   
   public void stop()
   {
      start = false;
   }

   public void destroy()
   {
      scheduledFuture.cancel(true);
   }

   @Override
   public void run()
   {
      if (!start)
         return;

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

            Pair<Point3D, PlanarRegion> intersectionPair = PlanarRegionTools.intersectRegionsWithRay(planarRegionsList, rayOrigin, rayDirection);
            FramePoint3D intersectionWidthWorld = new FramePoint3D(worldFrame, intersectionPair.getLeft());
            intersectionWidthWorld.changeFrame(cameraSensorFrame);

            pointCloud[widthIdx * numberWide + heightIdx] = intersectionWidthWorld;
            javafx.scene.paint.Color color = IdMappedColorFunction.INSTANCE.apply(intersectionPair.getRight().getRegionId());
            colors[widthIdx * numberWide + heightIdx] = new Color((int) (255.0 * color.getRed()), (int) (255.0 * color.getGreen()),
                                                                  (int) (255.0 * color.getBlue()), (int) (255.0 *color.getOpacity()));
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

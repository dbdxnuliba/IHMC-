package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.plotting.artifact.CircleArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.plotting.Plotter;

public class SpatialNodePlotter
{
   private ExploringDefinition spatialDefinition;

   private List<Plotter> plotters = new ArrayList<Plotter>();

   private int cnt;

   private boolean isFrameEnabled;

   public SpatialNodePlotter(ExploringDefinition spatialDefinition, boolean enabled)
   {
      this.spatialDefinition = spatialDefinition;
      SpatialData randomSpatialData = spatialDefinition.getRandomSpatialData();

      List<String> plotterNames = new ArrayList<String>();
      for (int i = 0; i < randomSpatialData.getNumberOfExploringRigidBodies(); i++)
      {
         plotterNames.add(randomSpatialData.getRigidBodyName(i) + "_px");
         plotterNames.add(randomSpatialData.getRigidBodyName(i) + "_py");
         plotterNames.add(randomSpatialData.getRigidBodyName(i) + "_pz");
         plotterNames.add(randomSpatialData.getRigidBodyName(i) + "_rvx");
         plotterNames.add(randomSpatialData.getRigidBodyName(i) + "_rvy");
         plotterNames.add(randomSpatialData.getRigidBodyName(i) + "_rvz");
      }

      for (int i = 0; i < plotterNames.size(); i++)
      {
         Plotter plotter = new Plotter();

         plotter.setPreferredSize(400, 600);

         plotter.setViewRange(2.0);
         plotter.setXYZoomEnabled(true);
         plotter.setShowLabels(true);
         plotter.setFocusPointX(0.5);
         plotter.setFocusPointY(0.0);
         plotters.add(plotter);

         String plotterName = plotterNames.get(i);
         isFrameEnabled = enabled;
         if (enabled)
            plotter.showInNewWindow(plotterName, false);
      }
      cnt = 0;
   }

   public void closeAll()
   {
      for (int i = 0; i < plotters.size(); i++)
      {
         // TODO : close panels.
         if (isFrameEnabled)
            plotters.get(i).getJFrame().dispose();
      }
   }

   public void update(List<SpatialNode> path, int type)
   {
      for (int i = 0; i < path.size(); i++)
         update(path.get(i), type);
   }

   /**
    * 1::node
    * 2::path
    * 3::shortcut
    */
   public void update(SpatialNode node, int type)
   {
      double progress = spatialDefinition.getExploringProgress(node);

      TDoubleArrayList configurationsList = getConfigurationsOfNode(node);

      Color color;
      double diameter = 0.01;

      for (int configurationIndex = 0; configurationIndex < configurationsList.size(); configurationIndex++)
      {
         String prefix;
         switch (type)
         {
         case 1:
            prefix = "" + cnt + "_node_" + configurationIndex;
            if (node.isValid())
               color = Color.blue;
            else
               color = Color.red;
            break;
         case 2:
            prefix = "" + cnt + "_path_" + configurationIndex;
            color = Color.black;
            break;
         case 3:
            prefix = "" + cnt + "_shortcut_" + configurationIndex;
            color = Color.green;
            break;
         default:
            prefix = "";
            color = Color.white;
            break;
         }

         double configurationData = configurationsList.get(configurationIndex);

         if (node.getParent() != null && node.isValid())
         {
            SpatialNode parentNode = node.getParent();
            double parentProgress = spatialDefinition.getExploringProgress(parentNode);
            double parentConfigurationData = getConfigurationsOfNode(parentNode).get(configurationIndex);

            LineArtifact lineArtifact = new LineArtifact(prefix + "_line", new Point2D(parentProgress, parentConfigurationData),
                                                         new Point2D(progress, configurationData));

            lineArtifact.setColor(color);
            plotters.get(configurationIndex).addArtifact(lineArtifact);
         }

         CircleArtifact nodeArtifact = new CircleArtifact(prefix + "_node", progress, configurationData, diameter, true);
         nodeArtifact.setColor(color);

         plotters.get(configurationIndex).addArtifact(nodeArtifact);
         plotters.get(configurationIndex).update();
      }

      cnt++;
   }

   private double[] getConfigurationsOfSpatial(RigidBodyTransform transform)
   {
      double[] configurations = new double[6];
      configurations[0] = transform.getTranslationX();
      configurations[1] = transform.getTranslationY();
      configurations[2] = transform.getTranslationZ();

      Vector3D rv = new Vector3D();
      RotationVectorConversion.convertMatrixToRotationVector(transform.getRotationMatrix(), rv);
      configurations[3] = rv.getX();
      configurations[4] = rv.getY();
      configurations[5] = rv.getZ();

      return configurations;
   }

   private TDoubleArrayList getConfigurationsOfNode(SpatialNode node)
   {
      TDoubleArrayList configurationsList = new TDoubleArrayList();

      for (int i = 0; i < node.getSpatialData().getRigidBodySpatials().size(); i++)
      {
         RigidBodyTransform spatial = node.getSpatialData(i);
         configurationsList.addAll(getConfigurationsOfSpatial(spatial));
      }

      return configurationsList;
   }
}

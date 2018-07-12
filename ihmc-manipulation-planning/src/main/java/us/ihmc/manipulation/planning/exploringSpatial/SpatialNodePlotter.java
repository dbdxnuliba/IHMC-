package us.ihmc.manipulation.planning.exploringSpatial;

import java.awt.Color;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.plotting.artifact.CircleArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.plotting.Plotter;

public class SpatialNodePlotter
{
   private boolean saving = false;

   private ExploringDefinition spatialDefinition;

   private List<Plotter> plotters = new ArrayList<Plotter>();

   private int cnt;

   private boolean isFrameEnabled;

   public SpatialNodePlotter(ExploringDefinition spatialDefinition, boolean enabled)
   {
      this.spatialDefinition = spatialDefinition;
      SpatialData randomSpatialData = spatialDefinition.createRandomSpatialData();

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

      //TEMP
      SpatialNode saveNode = new SpatialNode(node);
      switch (type)
      {
      case 1:
         if (saveNode.isValid())
            validNodes.add(saveNode);
         else
            inValidNodes.add(saveNode);
         break;
      case 2:
         path.add(saveNode);
         break;
      case 3:
         shortcutpath.add(saveNode);
         break;
      default:
         break;
      }

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

   // TEMP
   private List<SpatialNode> validNodes = new ArrayList<SpatialNode>();
   private List<SpatialNode> inValidNodes = new ArrayList<SpatialNode>();
   private List<SpatialNode> path = new ArrayList<SpatialNode>();
   private List<SpatialNode> shortcutpath = new ArrayList<SpatialNode>();

   public void saveNodes()
   {
      if (!saving)
         return;

      PrintTools.info("" + validNodes.size());
      PrintTools.info("" + inValidNodes.size());
      PrintTools.info("" + path.size());
      PrintTools.info("" + shortcutpath.size());

      for (int i = 0; i < validNodes.get(0).getSpatialData().getNumberOfExploringRigidBodies(); i++)
      {
         PrintTools.info("" + validNodes.get(0).getSpatialData().getRigidBodyName(i));
      }

      String FILENAME = "C:\\Users\\inhol\\Documents\\MATLAB\\savingdata.txt";
      BufferedWriter bw = null;
      FileWriter fw = null;

      try
      {
         String content = "";

         for (int i = 0; i < validNodes.size(); i++)
         {
            SpatialNode node = validNodes.get(i);

            double progress = spatialDefinition.getExploringProgress(node);
            TDoubleArrayList configurationsList = getConfigurationsOfNode(node);

            String nodedata = "1" + "\t" + progress;
            for (int j = 0; j < configurationsList.size(); j++)
            {
               nodedata = nodedata + "\t" + configurationsList.get(j);
            }

            String parentnodedata = "";
            if (validNodes.get(i).getParent() != null)
            {
               SpatialNode parentnode = validNodes.get(i).getParent();

               double parentprogress = spatialDefinition.getExploringProgress(parentnode);
               TDoubleArrayList parentconfigurationsList = getConfigurationsOfNode(parentnode);

               parentnodedata = parentnodedata + "\t" + parentprogress;
               for (int j = 0; j < parentconfigurationsList.size(); j++)
               {
                  parentnodedata = parentnodedata + "\t" + parentconfigurationsList.get(j);
               }
            }
            nodedata = nodedata + parentnodedata;
            nodedata = nodedata + "\n";

            content = content + nodedata;
         }

         for (int i = 0; i < inValidNodes.size(); i++)
         {
            SpatialNode node = inValidNodes.get(i);

            double progress = spatialDefinition.getExploringProgress(node);
            TDoubleArrayList configurationsList = getConfigurationsOfNode(node);

            String nodedata = "2" + "\t" + progress;
            for (int j = 0; j < configurationsList.size(); j++)
            {
               nodedata = nodedata + "\t" + configurationsList.get(j);
            }
            nodedata = nodedata + "\n";

            content = content + nodedata;
         }

         for (int i = 0; i < path.size(); i++)
         {
            SpatialNode node = path.get(i);

            double progress = spatialDefinition.getExploringProgress(node);
            TDoubleArrayList configurationsList = getConfigurationsOfNode(node);

            String nodedata = "3" + "\t" + progress;
            for (int j = 0; j < configurationsList.size(); j++)
            {
               nodedata = nodedata + "\t" + configurationsList.get(j);
            }
            nodedata = nodedata + "\n";

            content = content + nodedata;
         }

         for (int i = 0; i < shortcutpath.size(); i++)
         {
            SpatialNode node = shortcutpath.get(i);

            double progress = spatialDefinition.getExploringProgress(node);
            TDoubleArrayList configurationsList = getConfigurationsOfNode(node);

            String nodedata = "4" + "\t" + progress;
            for (int j = 0; j < configurationsList.size(); j++)
            {
               nodedata = nodedata + "\t" + configurationsList.get(j);
            }

            String parentnodedata = "";
            if (shortcutpath.get(i).getParent() != null)
            {
               SpatialNode parentnode = shortcutpath.get(i).getParent();

               double parentprogress = spatialDefinition.getExploringProgress(parentnode);
               TDoubleArrayList parentconfigurationsList = getConfigurationsOfNode(parentnode);

               parentnodedata = parentnodedata + "\t" + parentprogress;
               for (int j = 0; j < parentconfigurationsList.size(); j++)
               {
                  parentnodedata = parentnodedata + "\t" + parentconfigurationsList.get(j);
               }
            }
            nodedata = nodedata + parentnodedata;
            nodedata = nodedata + "\n";

            content = content + nodedata;
         }

         fw = new FileWriter(FILENAME);
         bw = new BufferedWriter(fw);
         bw.write(content);

         System.out.println("Done");

      }
      catch (IOException e)
      {

         e.printStackTrace();

      } finally
      {

         try
         {

            if (bw != null)
               bw.close();

            if (fw != null)
               fw.close();

         }
         catch (IOException ex)
         {

            ex.printStackTrace();

         }

      }
   }
}

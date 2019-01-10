package us.ihmc.manipulation.planning.planarRobot;

import java.awt.Color;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class PlanarRobotPathPlanningTest
{
   private final PlanarRobot robot;
   private final Vector2D fieldRange = new Vector2D(-10.0, 10.0);
   private PlanarRobotTask endEffectorTaskX;
   private PlanarRobotTask endEffectorTaskY;
   private PlanarRobotTask endEffectorTaskYaw;

   public PlanarRobotPathPlanningTest()
   {
      robot = new PlanarRobot();
   }

   public void testBuildingRobot()
   {
      System.out.println("testBuildingRobot");

      int numberOfJoints = 9;

      System.out.println("" + Math.PI * 10 / 180);
      robot.addBaseJoint(new Point2D(), 1.0, Math.PI * 10 / 180);

      for (int i = 0; i < numberOfJoints; i++)
      {
         robot.addJoint(1.0, Math.PI * 10 / 180);
      }

      robot.updateRobot();

      System.out.println("robot dimension is " + robot.getJointDimension());
      System.out.println("end effector position is " + robot.getLastJoint().getEndTip());
      System.out.println("end effector angle is " + robot.getLastJoint().getEndTipAngle());
   }

   public void testJointConfiguration()
   {
      System.out.println("testJointConfiguration");

      openRobotVisualizer(robot);
   }

   public void testTaskDefinition()
   {
      System.out.println("testTaskDefinition");

      endEffectorTaskX = new PlanarRobotTask("endX", robot)
      {
         @Override
         public double getTaskConfiguration(TDoubleArrayList jointConfiguration)
         {
            for (int i = 0; i < robot.getJointDimension(); i++)
               robot.getJoint(i).setJointAngle(jointConfiguration.get(i));
            robot.updateRobot();

            return robot.getLastJoint().getEndTip().getX();
         }
      };

      endEffectorTaskY = new PlanarRobotTask("endY", robot)
      {
         @Override
         public double getTaskConfiguration(TDoubleArrayList jointConfiguration)
         {
            for (int i = 0; i < robot.getJointDimension(); i++)
               robot.getJoint(i).setJointAngle(jointConfiguration.get(i));
            robot.updateRobot();

            return robot.getLastJoint().getEndTip().getY();
         }
      };

      endEffectorTaskYaw = new PlanarRobotTask("endYaw", robot)
      {
         @Override
         public double getTaskConfiguration(TDoubleArrayList jointConfiguration)
         {
            for (int i = 0; i < robot.getJointDimension(); i++)
               robot.getJoint(i).setJointAngle(jointConfiguration.get(i));
            robot.updateRobot();

            return robot.getLastJoint().getEndTipAngle();
         }
      };
      
      robot.addTask(endEffectorTaskX);
      robot.addTask(endEffectorTaskY);
      robot.addTask(endEffectorTaskYaw);
   }

   public void testForwardKinematics()
   {
      System.out.println("testForwardKinematics");

      TDoubleArrayList jointConfigurationOne = new TDoubleArrayList();
      for (int i = 0; i < robot.getJointDimension(); i++)
         jointConfigurationOne.add(0.0);

      robot.setJointConfiguration(jointConfigurationOne);
      TDoubleArrayList taskConfigurationOne = robot.getTaskConfiguration();
      System.out.println("TaskConfigurationOne ");
      for (int i = 0; i < taskConfigurationOne.size(); i++)
         System.out.println("" + taskConfigurationOne.get(i));

      TDoubleArrayList jointConfigurationTwo = new TDoubleArrayList();
      for (int i = 0; i < robot.getJointDimension(); i++)
         jointConfigurationTwo.add(Math.PI * 10 / 180);

      robot.setJointConfiguration(jointConfigurationTwo);
      TDoubleArrayList taskConfigurationTwo = robot.getTaskConfiguration();
      System.out.println("TaskConfigurationTwo ");
      for (int i = 0; i < taskConfigurationTwo.size(); i++)
         System.out.println("" + taskConfigurationTwo.get(i));
   }

   public void testInverseKinematics()
   {
      System.out.println("testInverseKinematics");
   }

   public static void main(String[] args)
   {
      PlanarRobotPathPlanningTest test = new PlanarRobotPathPlanningTest();
      test.testBuildingRobot();
      test.testJointConfiguration();
      test.testTaskDefinition();
      test.testForwardKinematics();
      test.testInverseKinematics();

      System.out.println("all tests are completed");
   }

   private void openRobotVisualizer(PlanarRobot robot)
   {
      XYSeriesCollection data = new XYSeriesCollection();

      for (int i = 0; i < robot.getJointDimension(); i++)
      {
         XYSeries link = new XYSeries(i + "link");
         link.add(robot.getJoint(i).getJointPoint().getX(), robot.getJoint(i).getJointPoint().getY());
         link.add(robot.getJoint(i).getEndTip().getX(), robot.getJoint(i).getEndTip().getY());
         data.addSeries(link);
      }

      JFreeChart chart = ChartFactory.createXYLineChart("Planar Robot", "X", "Y", data, PlotOrientation.VERTICAL, false, true, false);
      XYPlot xyPlot = chart.getXYPlot();
      xyPlot.getRangeAxis().setRange(fieldRange.getX(), fieldRange.getY());
      xyPlot.getDomainAxis().setRange(fieldRange.getX(), fieldRange.getY());

      XYLineAndShapeRenderer renderer = new XYLineAndShapeRenderer();
      for (int i = 0; i < robot.getJointDimension(); i++)
      {
         renderer.setSeriesPaint(i, Color.RED);
      }
      xyPlot.setRenderer(renderer);

      ChartPanel chartPanel = new ChartPanel(chart);
      chartPanel.setPreferredSize(new java.awt.Dimension(500, 500));

      JFrame frame;

      frame = new JFrame(this.getClass().getSimpleName());
      frame.setLocation(200, 100);

      frame.add(chartPanel);
      frame.pack();
      frame.setVisible(true);
   }

}

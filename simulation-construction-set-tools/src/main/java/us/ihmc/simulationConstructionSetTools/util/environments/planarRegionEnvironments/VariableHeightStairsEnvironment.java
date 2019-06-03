package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;



import java.util.ArrayList;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;


public class VariableHeightStairsEnvironment extends PlanarRegionEnvironmentInterface
{
   private ArrayList<Point3D> m_stepsCenters;

   public VariableHeightStairsEnvironment(ArrayList<Double> stepsHeights, double length, double width)
   {

      m_stepsCenters = new ArrayList<Point3D>();

      generator.identity();
      generator.addRectangle(length, width);
      m_stepsCenters.add(new Point3D(0.0, 0.0, 0.0));


      double currentHeight = 0.0;
      
      if (stepsHeights != null && !stepsHeights.isEmpty())
      {
         for (int i = 0; i < stepsHeights.size(); ++i)
         {
            double deltaHeight = stepsHeights.get(i) - currentHeight;
            generator.translate(length / 2.0, 0.0, deltaHeight / 2.0);
            generator.rotate(0.5 * Math.PI, Axis.Y);
            generator.addRectangle(deltaHeight, 2.0);
            generator.rotate(-0.5 * Math.PI, Axis.Y);
            generator.translate(length / 2.0, 0.0, deltaHeight / 2.0);
            generator.addRectangle(length, width);
            m_stepsCenters.add(new Point3D(length * (1 + i), 0.0, stepsHeights.get(i)));

            currentHeight = stepsHeights.get(i);
         }
      }

      addPlanarRegionsToTerrain(YoAppearance.Grey());
   }

   public VariableHeightStairsEnvironment(ArrayList<Double> stepsLocations, double length)
   {
      this(stepsLocations, length, 2.0);
   }

   public ArrayList<Point3D> getStepsCenter() 
   {
      return m_stepsCenters;
   }
}

package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotics.Assert.*;

import java.awt.Color;
import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;

public class ColorValueToRGBConversionTest extends ConcaveHullTestBasics
{
   private static final double EPS = 1.0e-4;
   private static final double rgbScale = 255.0;

   @Test
   public void testRandomColor()
   {
      Random random = new Random(0612L);
      int nTests = 0;
      for (int i = 0; i < ITERATIONS; i++)
      {
         int randomColorValue = random.nextInt();
         Color randomColor = new Color(randomColorValue);
         javafx.scene.paint.Color convertedRandomColor = StereoVisionPointCloudViewer.intToColor(randomColorValue);

         assertTrue("Red color conversion is working incorrect.", Math.abs(convertedRandomColor.getRed() * rgbScale - randomColor.getRed()) < EPS);
         assertTrue("Green color conversion is working incorrect.", Math.abs(convertedRandomColor.getGreen() * rgbScale - randomColor.getGreen()) < EPS);
         assertTrue("Blue color conversion is working incorrect.", Math.abs(convertedRandomColor.getBlue() * rgbScale - randomColor.getBlue()) < EPS);
         nTests++;
      }
      assert(nTests == ITERATIONS);
   }
   
	public static void main(String[] args)
	{
		MutationTestFacilitator.facilitateMutationTestForClass(ColorValueToRGBConversionTest.class, ColorValueToRGBConversionTest.class);
	}

}

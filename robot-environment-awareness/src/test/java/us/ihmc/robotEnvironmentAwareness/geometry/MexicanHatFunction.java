package us.ihmc.robotEnvironmentAwareness.geometry;

import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.apache.commons.lang3.mutable.MutableBoolean;
import org.junit.After;
import org.junit.Before;
import com.sun.javafx.application.PlatformImpl;
import javafx.stage.Stage;
import javafx.stage.Window;
import javafx.stage.WindowEvent;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerManager;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerVisualizerUI;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegionsList;

/* 
 * The Mexican Hat Function is used to construct a smooth hull with two pockets.
 * majority of the code is from Jeff Heaton:  https://github.com/jeffheaton/aifh/search?q=mexican&unscoped_q=mexican
 * 
*/

/**
* A function.  Returns a single scalar variable, accepts a vector of x.
*/
interface Fn
{
	/**
	* Evaluate the function.
	*
	* @param x A vector input.
	* @return The output from the function.
	*/
	double evaluate(double[] x);
}

/**
* A function that implements a radial basis function (RBF).
*/
interface FnRBF extends Fn
{

	/**
	* Get the center for the specified dimension.
	*
	* @param dimension The dimension.
	* @return The center.
	*/
	double getCenter(int dimension);

	/**
	* Set the center for the specified dimension.
	*
	* @param dimension The dimension.
	* @param value     The value to set the center.
	*/
	void setCenter(int dimension, double value);

	/**
	* @return The dimension count.
	*/
	int getDimensions();

	/**
	* @return The width.
	*/
	double getWidth();

	/**
	* Set the width.
	*
	* @param theWidth The width.
	*/
	void setWidth(double theWidth);
}

/**
* Provides the basics for an RBF function.  RBF functions take their "parameters" from a vector (and starting index).
* This allows many RBF's to be "stacked" together in a single vector.  RBF parameters are: a single width and a
* vector of centers.  Therefore the size required to store one RBF is (dimensions + 1).  There is no peak parameter,
* the peak is assumed to be 1.
*/
abstract class AbstractRBF implements FnRBF
{

	/**
	* The parameter vector.  Holds the RBF width and centers.  This vector may hold multiple RBF's.
	*/
	private final double[] params;

	/**
	* The index to the widths.
	*/
	private final int indexWidth;

	/**
	* The index to the centers.
	*/
	private final int indexCenters;

	/**
	* The dimensions.
	*/
	private final int dimensions;

	/**
	* Construct the RBF. Each RBF will require space equal to (dimensions + 1) in the params vector.
	*
	* @param theDimensions The number of dimensions.
	* @param theParams     A vector to hold the paramaters.
	* @param theIndex      The index into the params vector.  You can store multiple RBF's in a vector.
	*/
	public AbstractRBF(final int theDimensions, final double[] theParams, final int theIndex)
	{
		this.dimensions = theDimensions;
		this.params = theParams;
		this.indexWidth = theIndex;
		this.indexCenters = theIndex + 1;
	}

	/**
	* {@inheritDoc}
	*/
	@Override
	public final double getCenter(final int dimension)
	{
		return this.params[indexCenters + dimension];
	}

	/**
	* {@inheritDoc}
	*/
	@Override
	public final int getDimensions()
	{
		return this.dimensions;
	}

	/**
	* {@inheritDoc}
	*/
	@Override
	public final double getWidth()
	{
		return this.params[indexWidth];
	}

	/**
	* {@inheritDoc}
	*/
	@Override
	public final void setWidth(final double theWidth)
	{
		this.params[indexWidth] = theWidth;
	}

	/**
	* {@inheritDoc}
	*/
	@Override
	public String toString()
	{
		final NumberFormat f = NumberFormat.getNumberInstance();
		f.setMinimumFractionDigits(2);

		final StringBuilder result = new StringBuilder();
		result.append("[");
		result.append(this.getClass().getSimpleName());
		result.append(":width=");
		result.append(f.format(this.getWidth()));
		result.append(",center=");
		for (int i = 0; i < this.dimensions; i++)
		{
			if (i > 0)
			{
				result.append(",");
			}
			result.append(f.format(this.params[this.indexCenters + i]));

		}

		result.append("]");
		return result.toString();
	}

	/**
	* {@inheritDoc}
	*/
	@Override
	public void setCenter(final int dimension, final double value)
	{
		this.params[indexCenters + dimension] = value;
	}

	}

/**
 * The Mexican Hat, or Ricker wavelet, Radial Basis Function.
 * <p/>
 * It is usually only referred to as the "Mexican hat" in the Americas, due to
 * cultural association with the "sombrero". In technical nomenclature this
 * function is known as the Ricker wavelet, where it is frequently employed to
 * model seismic data.
 * <p/>
 * http://en.wikipedia.org/wiki/Mexican_Hat_Function
 */
public class MexicanHatFunction extends AbstractRBF
{

	
	/**
	 * Construct the Mexican Hat RBF. Each RBF will require space equal to (dimensions + 1) in the params vector.
	 *
	 * @param theDimensions The number of dimensions.
	 * @param theParams     A vector to hold the parameters.
	 * @param theIndex      The index into the params vector.  You can store multiple RBF's in a vector.
	 */
	public MexicanHatFunction(final int theDimensions, final double[] theParams, final int theIndex)
	{
		super(theDimensions, theParams, theIndex);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public double evaluate(final double[] x)
	{
		// calculate the "norm", but don't take square root
		// don't square because we are just going to square it
		double norm = 0;
		for (int i = 0; i < getDimensions(); i++)
		{
			final double center = this.getCenter(i);
			norm += Math.pow(x[i] - center, 2);
		}

		// calculate the value

		return (1 - norm) * Math.exp(-norm / 2);

		/*
		 *    function [psi,x] = mexihat(lb,ub,n)
		 * 	x = linspace(lb,ub,n);
		 *	   psi = (1-x.^2).*(2/(sqrt(3)*pi^0.25)) .* exp(-x.^2/2)  ;
		 */

	}

	
	public static void main(String[] args)
	{
		MutationTestFacilitator.facilitateMutationTestForClass(MexicanHatFunction.class, MexicanHatFunction.class);
	}
		


}

package us.ihmc.sensorProcessing.encoder.processors;

import us.ihmc.sensorProcessing.ProcessedPositionSensor;
import us.ihmc.sensorProcessing.ProcessedVelocitySensor;

import com.yobotics.simulationconstructionset.robotController.SensorProcessor;

/**
 * An EncoderProcessor should be constructed with the objects required to know the encoder raw state so that
 * the processor can estimate the processed state.
 */
public interface EncoderProcessor extends ProcessedPositionSensor, ProcessedVelocitySensor, SensorProcessor
{
}

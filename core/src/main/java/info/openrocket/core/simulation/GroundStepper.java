package info.openrocket.core.simulation;

import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Quaternion;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import info.openrocket.core.simulation.exception.SimulationException;

public class GroundStepper extends AbstractSimulationStepper {
	private static final Logger log = LoggerFactory.getLogger(GroundStepper.class);

	DataStore store = new DataStore();

	@Override
	public SimulationStatus initialize(SimulationStatus status) {
		log.trace("initializing GroundStepper");
		
		return status;
	}

	@Override
	public void step(SimulationStatus status, double timeStep) throws SimulationException {
		log.trace("step:  position=" + status.getRocketPosition() + ", velocity=" + status.getRocketVelocity());

		// set rocket orientation: laying flat on the ground on its side
		Quaternion q = new Quaternion(1, -1, 0, 0).normalizeIfNecessary();
		// set rocket acceleration
		Coordinate accWorldFrame = new Coordinate(0, 0, -9.80665);
		Coordinate accRocketFrame = q.rotate(accWorldFrame);
		status.setRocketOrientationQuaternion(q);
		status.setRocketAcceleration(accRocketFrame);
		status.setRocketRotationVelocity(new Coordinate(0, 0, 0));

		// for running real-time sim
		status.setSimulationTime(status.getSimulationTime() + GroundStepper.MIN_TIME_STEP);
		
		// Set to values sitting on the ground
		landedValues(status, store);
		
		// Put in a step to immediately go to landed status
//		double time = status.getSimulationTime();
//		if (timeStep > 2 * MIN_TIME_STEP) {
//			// Record timeStep in *current* flightDataBranch record, replacing NaN which was there
//			// previously
//			status.getFlightDataBranch().setValue(FlightDataType.TYPE_TIME_STEP, MIN_TIME_STEP);
//			timeStep = timeStep - MIN_TIME_STEP;
//
//			status.setSimulationTime(time + MIN_TIME_STEP);
//			status.storeData();
//			store.storeData(status);
//		}
		
		// Set status to reflect sitting on the ground ever since the last step
//		status.setSimulationTime(status.getSimulationTime() + timeStep);
		status.getFlightDataBranch().setValue(FlightDataType.TYPE_TIME_STEP, timeStep);
		
		status.storeData();
		store.storeData(status);
	}

	@Override
	void calculateAcceleration(SimulationStatus status, DataStore store) throws SimulationException {
		// empty
	}
}

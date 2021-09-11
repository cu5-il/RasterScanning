#include "myGlobals.h"
#include "constants.h"
#include "A3200.h"


bool setupDataCollection(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle) {
	// Adding the signals to be collected
	if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_AnalogOutput0, AXISINDEX_00, 0)) { return false; }	// AO0
	//if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_PositionFeedback, AXISINDEX_00, 0)) { return false; } // X-axis position feedback
	//if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_PositionFeedback, AXISINDEX_01, 0)) { return false; } // Y-axis position feedback
	//if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_PositionFeedback, AXISINDEX_02, 0)) { return false; } // Z-axis position feedback
	//if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_PositionFeedback, AXISINDEX_03, 0)) { return false; } // T-axis position feedback
	if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_AnalogInput0, AXISINDEX_01, 0)) { return false; }		// AI2
	if (!A3200DataCollectionConfigAddSignal(DCCHandle, DATASIGNAL_AnalogInput0, AXISINDEX_00, 0)) { return false; }		// AI0

	// Setting the parameters for the data collection
	if (!A3200DataCollectionConfigSetPeriod(DCCHandle, SAMPLING_TIME)) { return false; }
	if (!A3200DataCollectionConfigSetSamples(DCCHandle, NUM_DATA_SAMPLES)) { return false; }

	// REMOVE?:
	// Apply the data collection configuration to the A3200
	if (!A3200DataCollectionConfigApply(handle, DCCHandle)) { return false; }

	// Initializing AnalogOutput0 to 0V 
	if (!A3200IOAnalogOutput(handle, TASK_TRIG, 0, AXISINDEX_00, 0)) { return false; }

	return true;
}


/// \brief Gets the analog profile from the scanner.
///
/// This starts the data collection, sends a trigger signal to the scanner, then returns the collected data.
///
/// \param[in] handle The handle to the A3200
/// \param[in] DCCHandle The handle to an A3200 Data Collection Configuration object. If NULL, previous sent configuration will be used.
/// \param[out] data The retrieved sample point in format data[signal][sample].
/// \return TRUE on success, FALSE if an error occurred. Call A3200GetLastError() for more information.
bool collectData(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle, DOUBLE* data) {
	// Start the data collection
	if (!A3200DataCollectionStart(handle, DCCHandle)) { return false; }

	// Triggering the laser scanner by sending a pulse from AnalogOutput0 
	if (!A3200IOAnalogOutput(handle, TASK_TRIG, 0, AXISINDEX_00, OUT_VOLTAGE)) { return false; }
	if (!A3200IOAnalogOutput(handle, TASK_TRIG, 0, AXISINDEX_00, 0)) { return false; }

	// Retrieving the collected data
	if (!A3200DataCollectionDataRetrieve(handle, NUM_DATA_SIGNALS, NUM_DATA_SAMPLES, (DOUBLE*)data)) { return false; }

	return true;
}
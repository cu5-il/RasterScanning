/* Function declarations*/
#pragma once

#include "A3200.h"

bool setupDataCollection(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle);

bool collectData(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle, DOUBLE* data);


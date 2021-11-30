/* Function declarations*/
#pragma once
#include "A3200.h"

#ifndef SCANNER_FNS_H
#define SCANNER_FNS_H

bool setupDataCollection(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle);

bool collectData(A3200Handle handle, A3200DataCollectConfigHandle DCCHandle, DOUBLE* data);

#endif // SCANNER_FNS_H
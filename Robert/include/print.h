#pragma once
#include "myTypes.h"

#ifndef PRINT_H
#define PRINT_H

#define DISPOSAL_X 00
#define DISPOSAL_Y 153
#define SAFE_Z 5

void prePrint(Path firstWpt, PrintOptions printOpts);

void postPrint(Path lastWpt, PrintOptions printOpts);

#endif // !PRINT_H

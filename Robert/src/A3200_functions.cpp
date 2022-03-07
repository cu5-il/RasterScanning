#include "A3200_functions.h"
#include <iostream>
#include "A3200.h"

void A3200Error() {
	CHAR data[1024];
	A3200GetLastErrorString(data, 1024);
	std::cout << "A3200 Error: " << data << std::endl;
	system("pause");
}
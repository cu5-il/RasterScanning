#include "A3200_functions.h"
#include <iostream>
#include "A3200.h"
#include <thread>
#include<source_location>

void A3200Error(const std::source_location location) {
	CHAR data[1024];
	A3200GetLastErrorString(data, 1024);
	std::cout << "\a" 
		<< "Thread(" << std::this_thread::get_id() << ") "
		<< "File: " << location.file_name() << "("
		<< location.line() << ":"
		<< location.column() << ") `"
		<< location.function_name() << "`: "
		<< "A3200 Error: " << data << std::endl;
	//system("pause");
	Sleep(10);
}
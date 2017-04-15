/*
 * just_print.c
 *
 *  Created on: Apr 15, 2017
 *      Author: bulashevich
 */

#include <iostream>
#include "../protocol/protocol.h"

using namespace std;

class StreamProcessor
{
public:
	StreamProcessor(istream &is, ostream &os);
	void process();
protected:
	void virtual process_chunk() = 0;
};

int main()
{
	cout << "Hello, streamed processor!" << endl;
//	StreamProcessor proc(cin, cout);
//	proc.process();
	return 0;
}


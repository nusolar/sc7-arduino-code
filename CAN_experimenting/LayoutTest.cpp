/*
 * LayoutTest.cpp
 * Test for layouts.
 */
#ifndef COMPILE_ARDUINO // Don't use if we're compiling for the Arduino
/*
#include <iostream>
#include <stdint.h>
#include <bitset>

#include "Layouts.h"

using namespace std;

void test_layout(Layout& layout) {
	cout << endl;
	bitset<8> bits8;

	Frame layout_frame = layout.generate_frame();
	cout << "Frame id: 0x" << hex << layout_frame.id << dec << endl;
	bits8 = bitset<8>(layout_frame.dlc);
	cout << "Frame dlc: " << bits8 << endl;
	bits8 = bitset<8>(layout_frame.ide);
	cout << "Frame ide: " << bits8 << endl;
	bits8 = bitset<8>(layout_frame.rtr);
	cout << "Frame rtr: " << bits8 << endl;
	bits8 = bitset<8>(layout_frame.srr);
	cout << "Frame srr: " << bits8 << endl;
	cout << "Frame data low: " << layout_frame.low << endl;
	cout << "Frame data high: " << layout_frame.high << endl;
	for (int i = 0; i < 8; i++) {
		bits8 = bitset<8>(layout_frame.data[i]);
		cout << "Frame data byte " << i << ": " << bits8 << endl;
	}
}

int main() {
	Layout l;
	DriveCmd drive_cmd(100.0f, 50.0f);
	PowerCmd power_cmd(70.0f);

	Frame f;
	f.id = DRIVE_CMD_ID;
	f.dlc = 5;
	f.ide = 1;
	f.rtr = 3;
	f.srr = 2;
	f.low = 20;
	f.high = 80;
	DriveCmd drive_cmd2(f);

	test_layout(l);
	test_layout(drive_cmd);
	test_layout(drive_cmd2);
	test_layout(power_cmd);

	return 0;
}
*/
#endif

/*
 * LayoutTest.cpp
 * Test for layouts.
 */

#include <iostream>
#include <cstdint>

#include "Layouts.h"

using namespace std;

void test_layout(Layout*);

int main() {
	DriveCmd drive_cmd(100.0f, 50.0f);

	Frame drive_frame = drive_cmd.generate_frame();
	cout << "Frame id: " << drive_frame.id << endl;
	cout << "Frame dlc: " << drive_frame.dlc << endl;
	cout << "Frame ide: " << drive_frame.id << endl;
	cout << "Frame rtr: " << drive_frame.rtr << endl;
	cout << "Frame srr: " << drive_frame.srr << endl;
	cout << "Frame data low: " << drive_frame.low << endl;
	cout << "Frame data high: " << drive_frame.high << endl;
	for (int i = 0; i < 8; i++) {
		cout << "Frame data byte " << i << ": " << drive_frame.data[i] << endl;
	}

	test_layout(&drive_cmd);

	return 0;
}

void test_layout(Layout* layout) {
	cout << "\n\n" << endl;

	Frame layout_frame = layout->generate_frame();
	cout << "Frame id: " << layout_frame.id << endl;
	cout << "Frame dlc: " << layout_frame.dlc << endl;
	cout << "Frame ide: " << layout_frame.id << endl;
	cout << "Frame rtr: " << layout_frame.rtr << endl;
	cout << "Frame srr: " << layout_frame.srr << endl;
	cout << "Frame data low: " << layout_frame.low << endl;
	cout << "Frame data high: " << layout_frame.high << endl;
	for (int i = 0; i < 8; i++) {
		cout << "Frame data byte " << i << ": " << layout_frame.data[i] << endl;
	}
}
/*
 * CAN_test.cpp
 * Tests for CAN_IO lib.
 */

#include <iostream>
#include <bitset>

using std::cout;
using std::cin;
using std::endl;

typedef unsigned int byte; // won't compile with byte

// These need to be tested, but I believe they should work
byte CAN_IO::first_byte(unsigned int value) {
	return (value >> 4) & 0xFF; // Shift value over and return the top 8 bits.
}

byte CAN_IO::second_byte(unsigned int value) {
	return (value << 4) & 0xF0; // Return the bottom 4 bits of value, in the MSB position
}

int main() {
	bool done = false;
	int value, first, second;
	std::bitset<12> value_bits;
	std::bitset<8> first_bits, second_bits;
	char response = 'y';

	while (response == 'y') {
		cout << "Enter a value: ";
		cin >> std::hex >> value;
		value_bits = std::bitset<12>(value);
		first = first_byte(value);
		first_bits = std::bitset<8>(first);
		second = second_byte(value);
		second_bits = std::bitset<8>(second);
		cout << "Value: " << value_bits << endl;
		cout << "First byte: " << first_bits << endl;
		cout << "Second byte: " << second_bits << endl;
		cout << "Continue (y/n)? ";
		cin >> response;
	}

	return 0;
}
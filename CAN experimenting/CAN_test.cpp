/*
 * CAN_test.cpp
 * Tests for CAN_IO lib.
 */

#include <iostream>
#include <bitset>
#include <cstdint>

using namespace std;

uint8_t first_byte(unsigned int value) {
	return (value >> 3) & 0x00FF;
	// how does conversion to byte happen?
}

uint8_t second_byte(unsigned int value) {
	return (value << 5) & 0x00E0;
	// how does conversion to byte happen?
}

int main() {
	uint16_t value, first, second;
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
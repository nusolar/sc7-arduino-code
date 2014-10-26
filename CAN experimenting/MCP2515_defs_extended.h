/*
 * MCP2515_defs_extended.h
 * Contains additional register constants for the MCP2515.
 */

 #ifndef MCP2515_defs_extended_h
 #define MCP2515_defs_extended_h

 // filters for RX buffer 0
 #define RXF0SIDH 	0x00
 #define RXF0SIDL 	0x01
 #define RXF1SIDH 	0x04
 #define RXF1SIDL	0x05

 // filters for RX buffer 1
 #define RXF2SIDH 	0x08
 #define RXF2SIDL 	0x09
 #define RXF3SIDH 	0x10 // why not 0x0C?
 #define RXF3SIDL 	0x11
 #define RXF4SIDH 	0x14
 #define RXF4SIDL 	0x15
 #define RXF5SIDH 	0x18
 #define RXF5SIDL 	0x19

 // mask for RX buffer 0
 #define RXM0SIDH	0x20
 #define RXM0SIDL	0x21

 // mask for RX buffer 1
 #define RXM1SIDH	0x24
 #define RXM1SIDL	0x25


 #endif
/*
           Muskrat
      Møffenzeef Mødular 
        Røss Fish 2016 
  http://moffenzeefmodular.com
         CC-BY-NC-SA 
 
 Based on "Audio Sample Player"
      By: David Johnson-Davies 
http://www.technoblogy.com/show?QBB
          CC BY 4.0
*/


#include <avr/pgmspace.h>
#include <avr/power.h>

const unsigned char one_wav[] PROGMEM = {
  0x80, 0x85, 0x9d, 0xbc, 0xc7, 0xc5, 0xbb, 0xac, 0xa2, 0x9e, 0xa2, 0xa6,
  0xa4, 0x95, 0x7b, 0x5d, 0x45, 0x3b, 0x41, 0x4f, 0x5a, 0x57, 0x45, 0x2b,
  0x17, 0x19, 0x37, 0x6c, 0xa7, 0xd6, 0xeb, 0xe5, 0xcd, 0xb3, 0xa5, 0xa6,
  0xb3, 0xc0, 0xc1, 0xb3, 0x98, 0x79, 0x62, 0x58, 0x58, 0x5d, 0x60, 0x5a,
  0x4e, 0x40, 0x38, 0x3b, 0x49, 0x5e, 0x76, 0x88, 0x91, 0x90, 0x88, 0x7d,
  0x75, 0x75, 0x7f, 0x91, 0xa6, 0xb8, 0xc2, 0xc3, 0xbd, 0xb4, 0xad, 0xa8,
  0xa2, 0x98, 0x85, 0x6c, 0x53, 0x43, 0x41, 0x4e, 0x5f, 0x69, 0x62, 0x47,
  0x23, 0x07, 0x04, 0x23, 0x5e, 0xa4, 0xdf, 0xfd, 0xf9, 0xdd, 0xb8, 0x9d,
  0x95, 0x9f, 0xb0, 0xbd, 0xbb, 0xab, 0x92, 0x78, 0x66, 0x5b, 0x56, 0x51,
  0x4a, 0x42, 0x3c, 0x3e, 0x48, 0x5a, 0x6f, 0x81, 0x8b, 0x8b, 0x83, 0x78,
  0x70, 0x6f, 0x78, 0x8a, 0xa2, 0xb7, 0xc5, 0xc7, 0xbf, 0xb1, 0xa4, 0x9e,
  0xa0, 0xa5, 0xa6, 0x9b, 0x84, 0x66, 0x4b, 0x3c, 0x3e, 0x4b, 0x58, 0x5a,
  0x4c, 0x33, 0x1b, 0x16, 0x2b, 0x5a, 0x95, 0xcb, 0xe3, 0xd6, 0xba, 0x9f,
  0x8f, 0x89, 0x85, 0x80, 0x4c, 0x49, 0x53, 0x54, 0x1a, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x30, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x02, 0x00, 0x00, 0x00,
  0x31, 0x00, 0x69, 0x64, 0x33, 0x20, 0x24, 0x00, 0x00, 0x00, 0x49, 0x44,
  0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x52, 0x43, 0x4b,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x31, 0x54, 0x49, 0x54, 0x32,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x31, 0x30, 0x72
};

const unsigned char two_wav[] PROGMEM = {
  0x80, 0x90, 0xc8, 0xfe, 0xf7, 0xd8, 0xb7, 0xa0, 0x9d, 0xaa, 0xbc, 0xc6,
  0xc3, 0xb0, 0x98, 0x80, 0x6f, 0x66, 0x61, 0x5c, 0x54, 0x4c, 0x47, 0x49,
  0x54, 0x66, 0x79, 0x88, 0x8e, 0x8c, 0x82, 0x76, 0x6e, 0x6e, 0x77, 0x8a,
  0x9f, 0xb2, 0xbc, 0xbb, 0xb1, 0xa3, 0x97, 0x92, 0x95, 0x99, 0x98, 0x8b,
  0x73, 0x56, 0x3e, 0x33, 0x38, 0x46, 0x53, 0x53, 0x43, 0x2b, 0x17, 0x18,
  0x33, 0x66, 0xa1, 0xd2, 0xeb, 0xe8, 0xd3, 0xbb, 0xac, 0xad, 0xb9, 0xc7,
  0xcb, 0xbe, 0xa5, 0x88, 0x71, 0x65, 0x64, 0x69, 0x6c, 0x67, 0x5b, 0x4d,
  0x43, 0x44, 0x4f, 0x62, 0x78, 0x8a, 0x92, 0x91, 0x89, 0x7d, 0x73, 0x71,
  0x79, 0x88, 0x9c, 0xad, 0xb7, 0xb8, 0xb2, 0xa9, 0xa2, 0x9c, 0x97, 0x8d,
  0x7c, 0x64, 0x4b, 0x3b, 0x38, 0x43, 0x55, 0x61, 0x5d, 0x46, 0x24, 0x08,
  0x03, 0x1e, 0x57, 0x9c, 0xd9, 0xfb, 0xfc, 0xe3, 0xc0, 0xa5, 0x9c, 0xa5,
  0xb7, 0xc5, 0xc5, 0xb7, 0xa0, 0x87, 0x74, 0x68, 0x62, 0x5e, 0x57, 0x4f,
  0x48, 0x48, 0x50, 0x60, 0x73, 0x84, 0x8d, 0x8d, 0x85, 0x7b, 0x75, 0x75,
  0x7b, 0x80, 0x83, 0x7e, 0x4c, 0x49, 0x53, 0x54, 0x1a, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x31, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x02, 0x00, 0x00, 0x00,
  0x32, 0x00, 0x69, 0x64, 0x33, 0x20, 0x24, 0x00, 0x00, 0x00, 0x49, 0x44,
  0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x52, 0x43, 0x4b,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x32, 0x54, 0x49, 0x54, 0x32,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x31, 0x31, 0x72
};

const unsigned char three_wav[] PROGMEM = {
  0x7f, 0x8a, 0x76, 0x3a, 0x3e, 0x79, 0x97, 0x7a, 0x5f, 0x84, 0xb9, 0xae,
  0x6f, 0x5f, 0x9b, 0xc8, 0x9f, 0x57, 0x52, 0x8d, 0xaa, 0x7c, 0x42, 0x53,
  0x99, 0xb4, 0x7e, 0x49, 0x68, 0xb3, 0xc1, 0x84, 0x5b, 0x79, 0x9d, 0x84,
  0x50, 0x4d, 0x78, 0x83, 0x50, 0x23, 0x45, 0x97, 0xb4, 0x86, 0x70, 0xb6,
  0xfd, 0xcf, 0x5f, 0x47, 0x98, 0xba, 0x60, 0x05, 0x22, 0x7c, 0x8f, 0x5d,
  0x53, 0x99, 0xda, 0xc9, 0x8c, 0x7a, 0xa0, 0xb8, 0x93, 0x64, 0x70, 0x9c,
  0x94, 0x54, 0x3b, 0x76, 0xb5, 0x9f, 0x5b, 0x50, 0x90, 0xc1, 0xa1, 0x61,
  0x5c, 0x96, 0xb5, 0x81, 0x3e, 0x48, 0x8c, 0xa2, 0x6c, 0x40, 0x61, 0x99,
  0x98, 0x70, 0x73, 0xab, 0xcd, 0xa7, 0x6b, 0x6a, 0x9b, 0x9f, 0x53, 0x13,
  0x36, 0x7e, 0x6c, 0x17, 0x17, 0x94, 0xfa, 0xd7, 0x84, 0x8e, 0xda, 0xe2,
  0x92, 0x56, 0x6e, 0x9a, 0x86, 0x47, 0x34, 0x65, 0x95, 0x87, 0x62, 0x72,
  0xad, 0xba, 0x82, 0x5b, 0x84, 0xc2, 0xb4, 0x6a, 0x4a, 0x79, 0xaa, 0x90,
  0x4f, 0x44, 0x83, 0xb6, 0x94, 0x52, 0x54, 0x9f, 0xc4, 0x95, 0x6b, 0x74,
  0x89, 0x84, 0x7c, 0x7f, 0x4c, 0x49, 0x53, 0x54, 0x1a, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x32, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x02, 0x00, 0x00, 0x00,
  0x33, 0x00, 0x69, 0x64, 0x33, 0x20, 0x24, 0x00, 0x00, 0x00, 0x49, 0x44,
  0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x52, 0x43, 0x4b,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x33, 0x54, 0x49, 0x54, 0x32,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x31, 0x32, 0x72
};

const unsigned char four_wav[] PROGMEM = {
  0x80, 0x82, 0xbd, 0xe3, 0x93, 0x54, 0x6c, 0x9a, 0x89, 0x4c, 0x38, 0x63,
  0x91, 0x88, 0x69, 0x73, 0xa4, 0xb3, 0x88, 0x64, 0x82, 0xb6, 0xb0, 0x73,
  0x52, 0x74, 0xa1, 0x91, 0x58, 0x48, 0x7e, 0xb0, 0x95, 0x57, 0x57, 0x9c,
  0xc3, 0x97, 0x61, 0x6f, 0x9a, 0x90, 0x5a, 0x49, 0x72, 0x8a, 0x61, 0x29,
  0x37, 0x84, 0xb2, 0x91, 0x6c, 0x99, 0xec, 0xe6, 0x7f, 0x42, 0x7a, 0xba,
  0x89, 0x20, 0x0d, 0x5e, 0x95, 0x76, 0x4f, 0x75, 0xc4, 0xd9, 0xa5, 0x75,
  0x88, 0xb2, 0xa9, 0x74, 0x62, 0x8a, 0x9f, 0x6f, 0x3e, 0x5c, 0xa3, 0xad,
  0x71, 0x4e, 0x7a, 0xb4, 0xac, 0x73, 0x5c, 0x85, 0xac, 0x91, 0x54, 0x46,
  0x77, 0x9b, 0x7e, 0x4f, 0x57, 0x88, 0x99, 0x7c, 0x6e, 0x96, 0xc5, 0xb7,
  0x7b, 0x62, 0x8b, 0xaa, 0x74, 0x20, 0x21, 0x70, 0x86, 0x35, 0x04, 0x62,
  0xe7, 0xf0, 0x95, 0x77, 0xbf, 0xec, 0xb1, 0x5f, 0x5d, 0x90, 0x96, 0x5e,
  0x36, 0x52, 0x87, 0x90, 0x71, 0x6a, 0x95, 0xb5, 0x98, 0x69, 0x72, 0xa9,
  0xba, 0x87, 0x55, 0x64, 0x97, 0x9d, 0x69, 0x45, 0x6a, 0xa0, 0x99, 0x73,
  0x6b, 0x81, 0x87, 0x80, 0x4c, 0x49, 0x53, 0x54, 0x1a, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x33, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x02, 0x00, 0x00, 0x00,
  0x34, 0x00, 0x69, 0x64, 0x33, 0x20, 0x24, 0x00, 0x00, 0x00, 0x49, 0x44,
  0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x52, 0x43, 0x4b,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x34, 0x54, 0x49, 0x54, 0x32,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x31, 0x33, 0x72

};

const unsigned char five_wav[] PROGMEM = {
  0x7f, 0x80, 0x81, 0x7e, 0x7c, 0x80, 0x84, 0x83, 0x7f, 0x7f, 0x83, 0x87,
  0x85, 0x80, 0x81, 0x87, 0x89, 0x85, 0x7f, 0x81, 0x88, 0x8a, 0x82, 0x7c,
  0x81, 0x89, 0x88, 0x7d, 0x78, 0x80, 0x88, 0x81, 0x6e, 0x68, 0x79, 0x85,
  0x67, 0x28, 0x03, 0x2d, 0x95, 0xed, 0xf8, 0xc0, 0x87, 0x79, 0x8c, 0x97,
  0x8a, 0x79, 0x78, 0x83, 0x87, 0x7e, 0x75, 0x78, 0x81, 0x82, 0x7a, 0x75,
  0x79, 0x7f, 0x7f, 0x78, 0x76, 0x7a, 0x7f, 0x7e, 0x79, 0x78, 0x7e, 0x81,
  0x7f, 0x7b, 0x7c, 0x81, 0x83, 0x80, 0x7d, 0x7f, 0x83, 0x84, 0x81, 0x7f,
  0x81, 0x85, 0x85, 0x81, 0x80, 0x83, 0x85, 0x84, 0x80, 0x7f, 0x82, 0x84,
  0x82, 0x7e, 0x7e, 0x82, 0x83, 0x7f, 0x7c, 0x7d, 0x81, 0x81, 0x7d, 0x7b,
  0x7d, 0x80, 0x7f, 0x7b, 0x7a, 0x7c, 0x7f, 0x7e, 0x7a, 0x7a, 0x7e, 0x80,
  0x7e, 0x7b, 0x7c, 0x80, 0x82, 0x7f, 0x7c, 0x7e, 0x83, 0x84, 0x80, 0x7e,
  0x82, 0x87, 0x86, 0x81, 0x80, 0x85, 0x89, 0x87, 0x80, 0x80, 0x86, 0x8a,
  0x85, 0x7d, 0x7e, 0x87, 0x89, 0x81, 0x78, 0x7c, 0x87, 0x84, 0x77, 0x72,
  0x7a, 0x81, 0x7e, 0x7f, 0x4c, 0x49, 0x53, 0x54, 0x1a, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x02, 0x00, 0x00, 0x00,
  0x35, 0x00, 0x69, 0x64, 0x33, 0x20, 0x24, 0x00, 0x00, 0x00, 0x49, 0x44,
  0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x52, 0x43, 0x4b,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x35, 0x54, 0x49, 0x54, 0x32,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x31, 0x34, 0x72
};

const unsigned char six_wav[] PROGMEM = {
  0x81, 0x8a, 0x64, 0xce, 0x4c, 0x76, 0xb6, 0x35, 0x8b, 0x78, 0xa5, 0x3a,
  0xa4, 0xbc, 0x51, 0xa8, 0x55, 0x92, 0x6a, 0x9d, 0x60, 0x2c, 0xfe, 0x61,
  0x7d, 0x88, 0x7e, 0x99, 0x5f, 0xa7, 0x22, 0x9d, 0xa6, 0x55, 0x8e, 0x8b,
  0xb3, 0x37, 0xbb, 0x7c, 0x42, 0xb5, 0x3d, 0x96, 0x7f, 0xad, 0x65, 0x55,
  0xf2, 0x4d, 0x79, 0x6b, 0x76, 0x80, 0x85, 0x9a, 0x22, 0xde, 0x96, 0x60,
  0x88, 0x6c, 0xa4, 0x34, 0xb7, 0x5b, 0x5e, 0xd5, 0x50, 0x97, 0x89, 0xa7,
  0x41, 0x71, 0xbc, 0x39, 0x9a, 0x61, 0x94, 0x8c, 0xa1, 0x90, 0x18, 0xeb,
  0x6c, 0x51, 0x81, 0x66, 0xa5, 0x67, 0xbe, 0x48, 0x8e, 0xc3, 0x49, 0x7d,
  0x6d, 0xa6, 0x34, 0x9e, 0xa9, 0x42, 0xd0, 0x59, 0x8d, 0x83, 0x8f, 0x6a,
  0x25, 0xe4, 0x61, 0x73, 0x8b, 0x81, 0x9d, 0x81, 0xa7, 0x12, 0xab, 0x9f,
  0x44, 0x8c, 0x6b, 0xbe, 0x51, 0xb5, 0x86, 0x3f, 0xcf, 0x44, 0x74, 0x7d,
  0x9e, 0x5e, 0x66, 0xe1, 0x53, 0x99, 0x6e, 0x76, 0x84, 0x79, 0x9a, 0x07,
  0xda, 0xa0, 0x56, 0x9f, 0x6a, 0xa9, 0x5c, 0xa2, 0x4d, 0x60, 0xb5, 0x65,
  0x84, 0x7f, 0x88, 0x7e, 0x4c, 0x49, 0x53, 0x54, 0x1a, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x35, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x02, 0x00, 0x00, 0x00,
  0x36, 0x00, 0x69, 0x64, 0x33, 0x20, 0x24, 0x00, 0x00, 0x00, 0x49, 0x44,
  0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x52, 0x43, 0x4b,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x36, 0x54, 0x49, 0x54, 0x32,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x31, 0x35, 0x72
};

const unsigned char seven_wav[] PROGMEM = {
  0x85, 0x81, 0x72, 0x2b, 0x5b, 0x80, 0x9d, 0xac, 0x8c, 0xc8, 0xac, 0xa9,
  0x40, 0x57, 0x81, 0xb3, 0x8e, 0x58, 0x57, 0x60, 0x9b, 0x8e, 0x9e, 0x46,
  0x8c, 0x9c, 0xb7, 0x73, 0x57, 0x72, 0x79, 0x7f, 0x3f, 0x54, 0x56, 0x9c,
  0x4e, 0x2c, 0x16, 0xab, 0xf5, 0xc0, 0x8d, 0x70, 0xbd, 0xab, 0xb0, 0x6d,
  0x93, 0x91, 0xa9, 0x62, 0x4c, 0x6d, 0x93, 0xa4, 0x4f, 0x7a, 0x71, 0xb5,
  0xa0, 0x98, 0x51, 0x60, 0x95, 0xb9, 0x97, 0x3d, 0x50, 0x43, 0x7d, 0x47,
  0x7e, 0x8a, 0xc6, 0xbf, 0x7c, 0x62, 0x93, 0xea, 0xae, 0x7d, 0x40, 0x99,
  0x81, 0x7e, 0x3a, 0x44, 0x61, 0x65, 0x7a, 0x4a, 0x76, 0x7c, 0x96, 0x50,
  0x82, 0x95, 0xcc, 0xa6, 0x77, 0x56, 0x30, 0x78, 0x82, 0xae, 0x66, 0x86,
  0x92, 0xb1, 0x88, 0x97, 0xa9, 0xb7, 0xc3, 0x73, 0x7e, 0x6b, 0xbf, 0x76,
  0x41, 0x1e, 0x76, 0xa5, 0x72, 0x40, 0x3b, 0x84, 0x86, 0xbb, 0x83, 0xbf,
  0xb2, 0xb4, 0x5f, 0x41, 0x7a, 0xa1, 0xaa, 0x5b, 0x5f, 0x4f, 0x95, 0x8a,
  0xa7, 0x59, 0x6e, 0xa0, 0xad, 0x95, 0x4e, 0x72, 0x70, 0x89, 0x5d, 0x64,
  0x6d, 0x81, 0x80, 0x7e, 0x4c, 0x49, 0x53, 0x54, 0x1a, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x36, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x02, 0x00, 0x00, 0x00,
  0x37, 0x00, 0x69, 0x64, 0x33, 0x20, 0x24, 0x00, 0x00, 0x00, 0x49, 0x44,
  0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x52, 0x43, 0x4b,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x37, 0x54, 0x49, 0x54, 0x32,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x31, 0x36, 0x72
};

const unsigned char eight_wav[] PROGMEM = {
  0x80, 0x7d, 0x73, 0x75, 0x8c, 0xa8, 0xb6, 0xab, 0x89, 0x5e, 0x3e, 0x36,
  0x49, 0x67, 0x7c, 0x77, 0x57, 0x2a, 0x07, 0x03, 0x21, 0x57, 0x8c, 0xa7,
  0x9e, 0x7a, 0x4e, 0x31, 0x31, 0x49, 0x69, 0x7c, 0x74, 0x54, 0x2b, 0x0e,
  0x0a, 0x1e, 0x3e, 0x58, 0x61, 0x59, 0x4c, 0x49, 0x58, 0x76, 0x95, 0xa2,
  0x96, 0x75, 0x4e, 0x37, 0x3b, 0x57, 0x79, 0x8e, 0x87, 0x66, 0x3d, 0x24,
  0x2c, 0x56, 0x92, 0xc6, 0xdd, 0xd0, 0xab, 0x84, 0x70, 0x7a, 0x99, 0xba,
  0xc9, 0xbd, 0x9b, 0x75, 0x5f, 0x62, 0x7b, 0x9c, 0xb3, 0xb8, 0xae, 0xa2,
  0xa2, 0xb5, 0xd4, 0xef, 0xf6, 0xe2, 0xbc, 0x96, 0x83, 0x8a, 0xa7, 0xc5,
  0xd1, 0xc0, 0x99, 0x6e, 0x58, 0x64, 0x91, 0xc9, 0xf4, 0xff, 0xe7, 0xba,
  0x92, 0x80, 0x8b, 0xa8, 0xc2, 0xc7, 0xb0, 0x87, 0x5f, 0x49, 0x4e, 0x66,
  0x82, 0x91, 0x8d, 0x7e, 0x70, 0x71, 0x84, 0xa0, 0xb4, 0xb2, 0x96, 0x6b,
  0x45, 0x36, 0x41, 0x5e, 0x78, 0x7c, 0x64, 0x38, 0x0f, 0x00, 0x14, 0x45,
  0x7d, 0xa2, 0xa5, 0x87, 0x5b, 0x37, 0x2e, 0x40, 0x61, 0x7a, 0x7b, 0x6e,
  0x63, 0x63, 0x71, 0x7f, 0x4c, 0x49, 0x53, 0x54, 0x1a, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x37, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x02, 0x00, 0x00, 0x00,
  0x38, 0x00, 0x69, 0x64, 0x33, 0x20, 0x24, 0x00, 0x00, 0x00, 0x49, 0x44,
  0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x52, 0x43, 0x4b,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x38, 0x54, 0x49, 0x54, 0x32,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x31, 0x37, 0x72

};

const unsigned char nine_wav[] PROGMEM = {
  0x81, 0x9c, 0xd6, 0xfb, 0xdf, 0xb1, 0x86, 0x69, 0x61, 0x64, 0x6d, 0x71,
  0x70, 0x6a, 0x67, 0x69, 0x6f, 0x76, 0x7a, 0x7c, 0x81, 0x8e, 0xa6, 0xc6,
  0xe2, 0xec, 0xd9, 0xaa, 0x6b, 0x31, 0x0f, 0x12, 0x37, 0x6e, 0xa3, 0xc4,
  0xcb, 0xbb, 0xa1, 0x8b, 0x81, 0x82, 0x8a, 0x90, 0x91, 0x8d, 0x89, 0x86,
  0x85, 0x81, 0x77, 0x67, 0x55, 0x4d, 0x57, 0x76, 0xa3, 0xce, 0xe4, 0xdb,
  0xb2, 0x77, 0x3f, 0x1e, 0x1d, 0x3b, 0x69, 0x93, 0xad, 0xb2, 0xa6, 0x93,
  0x84, 0x7c, 0x79, 0x78, 0x75, 0x70, 0x6e, 0x70, 0x77, 0x7d, 0x7c, 0x6f,
  0x57, 0x3f, 0x35, 0x42, 0x6a, 0xa1, 0xd5, 0xf1, 0xe9, 0xc0, 0x82, 0x45,
  0x1d, 0x12, 0x22, 0x41, 0x61, 0x77, 0x82, 0x85, 0x87, 0x8b, 0x92, 0x97,
  0x98, 0x94, 0x8f, 0x8e, 0x93, 0x9b, 0x9c, 0x8e, 0x6e, 0x41, 0x17, 0x01,
  0x0d, 0x3b, 0x7f, 0xc3, 0xf2, 0xff, 0xea, 0xc0, 0x92, 0x70, 0x62, 0x63,
  0x6a, 0x71, 0x70, 0x6c, 0x68, 0x68, 0x6d, 0x74, 0x79, 0x7b, 0x7e, 0x88,
  0x9e, 0xbc, 0xdb, 0xec, 0xe2, 0xbb, 0x80, 0x40, 0x1b, 0x23, 0x44, 0x6c,
  0x87, 0x90, 0x89, 0x80, 0x4c, 0x49, 0x53, 0x54, 0x1a, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x38, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x02, 0x00, 0x00, 0x00,
  0x39, 0x00, 0x69, 0x64, 0x33, 0x20, 0x24, 0x00, 0x00, 0x00, 0x49, 0x44,
  0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x52, 0x43, 0x4b,
  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x39, 0x54, 0x49, 0x54, 0x32,
  0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x31, 0x38, 0x72

};

const unsigned char ten_wav[] PROGMEM = {
  0x7f, 0x7f, 0x77, 0x65, 0x4c, 0x31, 0x19, 0x0e, 0x11, 0x21, 0x39, 0x51,
  0x67, 0x7a, 0x8b, 0x9b, 0xab, 0xb7, 0xbc, 0xbb, 0xb5, 0xad, 0xa6, 0x9f,
  0x93, 0x7f, 0x61, 0x3d, 0x1c, 0x06, 0x00, 0x09, 0x1b, 0x2d, 0x3a, 0x41,
  0x45, 0x4f, 0x61, 0x7b, 0x99, 0xb3, 0xc5, 0xcb, 0xc9, 0xc0, 0xb4, 0xa4,
  0x91, 0x79, 0x60, 0x4a, 0x3b, 0x34, 0x33, 0x34, 0x32, 0x2d, 0x28, 0x2c,
  0x3d, 0x5c, 0x83, 0xaa, 0xc7, 0xd6, 0xd8, 0xd3, 0xcd, 0xcb, 0xcb, 0xc9,
  0xc1, 0xb1, 0x9a, 0x81, 0x6a, 0x57, 0x48, 0x3d, 0x36, 0x35, 0x3d, 0x50,
  0x6b, 0x89, 0xa3, 0xb4, 0xbd, 0xc1, 0xc8, 0xd5, 0xe7, 0xf8, 0xff, 0xf6,
  0xde, 0xbb, 0x98, 0x7b, 0x68, 0x5d, 0x56, 0x50, 0x49, 0x44, 0x44, 0x4b,
  0x57, 0x67, 0x78, 0x8a, 0x9d, 0xb3, 0xcb, 0xe1, 0xf0, 0xf1, 0xe3, 0xca,
  0xae, 0x96, 0x87, 0x7f, 0x77, 0x6a, 0x55, 0x39, 0x20, 0x10, 0x0e, 0x1b,
  0x31, 0x4a, 0x60, 0x74, 0x85, 0x96, 0xa6, 0xb3, 0xbb, 0xbc, 0xb7, 0xaf,
  0xa9, 0xa1, 0x98, 0x86, 0x6b, 0x48, 0x26, 0x0a, 0x05, 0x1c, 0x37, 0x50,
  0x62, 0x6e, 0x78, 0x7f, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x39, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x30, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00, 0x00, 0x00,
  0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x54, 0x52,
  0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x31, 
};

const unsigned char eleven_wav[] PROGMEM = {
  0x80, 0x83, 0x88, 0x8c, 0x8d, 0x8e, 0x8e, 0x8e, 0x8e, 0x8d, 0x8d, 0x8c,
  0x8a, 0x89, 0x87, 0x85, 0x83, 0x81, 0x7f, 0x7d, 0x7b, 0x79, 0x77, 0x76,
  0x74, 0x73, 0x72, 0x71, 0x71, 0x71, 0x71, 0x72, 0x72, 0x73, 0x74, 0x76,
  0x77, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7d, 0x7d, 0x7c, 0x7b, 0x79, 0x76,
  0x72, 0x6e, 0x68, 0x62, 0x5b, 0x54, 0x4c, 0x44, 0x3b, 0x32, 0x2a, 0x21,
  0x1a, 0x13, 0x0c, 0x07, 0x03, 0x01, 0x00, 0x00, 0x03, 0x07, 0x0d, 0x14,
  0x1e, 0x29, 0x35, 0x42, 0x51, 0x60, 0x70, 0x80, 0x90, 0x9f, 0xaf, 0xbd,
  0xca, 0xd7, 0xe2, 0xeb, 0xf2, 0xf8, 0xfc, 0xff, 0xff, 0xfe, 0xfc, 0xf8,
  0xf3, 0xed, 0xe5, 0xde, 0xd5, 0xcd, 0xc4, 0xbb, 0xb3, 0xab, 0xa3, 0x9d,
  0x97, 0x91, 0x8d, 0x89, 0x86, 0x84, 0x83, 0x82, 0x82, 0x82, 0x83, 0x84,
  0x85, 0x86, 0x88, 0x89, 0x8b, 0x8c, 0x8d, 0x8d, 0x8e, 0x8e, 0x8e, 0x8e,
  0x8d, 0x8c, 0x8b, 0x89, 0x88, 0x86, 0x84, 0x82, 0x80, 0x7e, 0x7c, 0x7a,
  0x78, 0x76, 0x75, 0x73, 0x72, 0x72, 0x71, 0x71, 0x72, 0x74, 0x76, 0x79,
  0x7b, 0x7d, 0x7e, 0x7f, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x72, 0x00, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x31, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x24, 0x00, 0x00, 0x00,
  0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1a, 0x54, 0x52,
  0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x31, 
};

const unsigned char twelve_wav[] PROGMEM = {
  0x7f, 0x80, 0x89, 0x94, 0x8f, 0x7e, 0x6b, 0x61, 0x69, 0x80, 0x99, 0xa5,
  0x9c, 0x81, 0x65, 0x56, 0x60, 0x7b, 0x99, 0xa8, 0xa0, 0x86, 0x69, 0x5a,
  0x61, 0x78, 0x91, 0x9e, 0x99, 0x87, 0x74, 0x6b, 0x6f, 0x7b, 0x85, 0x88,
  0x84, 0x80, 0x80, 0x84, 0x88, 0x86, 0x7b, 0x6d, 0x67, 0x71, 0x88, 0xa0,
  0xa9, 0x99, 0x76, 0x52, 0x45, 0x59, 0x88, 0xb8, 0xcb, 0xb2, 0x79, 0x3e,
  0x25, 0x3f, 0x82, 0xc7, 0xe7, 0xcc, 0x83, 0x35, 0x0e, 0x28, 0x75, 0xcb,
  0xf9, 0xe2, 0x92, 0x36, 0x02, 0x14, 0x64, 0xc2, 0xfb, 0xef, 0xa4, 0x47,
  0x0b, 0x12, 0x56, 0xaf, 0xeb, 0xe8, 0xab, 0x59, 0x20, 0x1f, 0x53, 0x9d,
  0xd2, 0xd6, 0xaa, 0x6b, 0x3d, 0x37, 0x59, 0x8c, 0xb3, 0xb9, 0x9f, 0x79,
  0x5b, 0x56, 0x68, 0x82, 0x95, 0x98, 0x8d, 0x7f, 0x77, 0x78, 0x7d, 0x80,
  0x7e, 0x79, 0x77, 0x7e, 0x8a, 0x93, 0x92, 0x84, 0x70, 0x62, 0x65, 0x78,
  0x92, 0xa3, 0xa1, 0x8b, 0x6d, 0x58, 0x5a, 0x72, 0x91, 0xa6, 0xa5, 0x8f,
  0x71, 0x5d, 0x5d, 0x70, 0x8a, 0x9c, 0x9c, 0x8d, 0x79, 0x70, 0x73, 0x7b,
  0x81, 0x82, 0x80, 0x7f, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x32, 0x30, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00, 0x00, 0x00,
  0x31, 0x32, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00, 0x00, 0x00,
  0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x54, 0x52,
  0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x31, 0x32, 0x54,
  0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 

};

const unsigned char thirteen_wav[] PROGMEM = {
  0x74, 0xb2, 0x01, 0x80, 0x73, 0x91, 0xc4, 0x8a, 0x73, 0x95, 0x48, 0x93,
  0x4d, 0x9b, 0x3e, 0x9b, 0xc5, 0xa1, 0x0b, 0xa3, 0x77, 0xa8, 0xa4, 0xaa,
  0x27, 0xaf, 0xef, 0xb1, 0xba, 0xb5, 0xe9, 0xb8, 0x34, 0xbc, 0xbb, 0xbf,
  0xb9, 0xc2, 0x97, 0xc6, 0x7d, 0xc9, 0x9e, 0xcd, 0x86, 0xd0, 0xd3, 0xd4,
  0xc0, 0xd7, 0x18, 0xdc, 0xfb, 0xde, 0x34, 0xe3, 0x02, 0xe6, 0x04, 0xea,
  0xc1, 0xec, 0x9c, 0xf0, 0x58, 0xf3, 0x2a, 0xf7, 0x06, 0xfa, 0xe6, 0xfd,
  0xfd, 0x00, 0xfe, 0x04, 0x49, 0x08, 0x5b, 0x0c, 0xb5, 0x0f, 0xbb, 0x13,
  0xf8, 0x16, 0xc7, 0x1a, 0xd4, 0x1d, 0x58, 0x21, 0x47, 0x24, 0x92, 0x27,
  0x9e, 0x2a, 0xdc, 0x2d, 0x53, 0x31, 0xad, 0x34, 0xc2, 0x38, 0x3b, 0x3c,
  0xe6, 0x40, 0x39, 0x44, 0x2a, 0x49, 0xd0, 0x4b, 0x75, 0x50, 0xca, 0x51,
  0x80, 0x55, 0x01, 0x55, 0x4b, 0x57, 0xc5, 0x54, 0x78, 0x55, 0x1f, 0x51,
  0x67, 0x50, 0xda, 0x4a, 0x17, 0x49, 0x28, 0x43, 0xb9, 0x40, 0x5b, 0x3b,
  0x4c, 0x38, 0x51, 0x34, 0x39, 0x30, 0x79, 0x2e, 0x37, 0x28, 0x3d, 0x2a,
  0x7f, 0x1e, 0xaa, 0x2a, 0xab, 0xcd, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x33, 0x34, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x33,
  0x34, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00,
};

const unsigned char fourteen_wav[] PROGMEM = {
  0x29, 0xf1, 0xae, 0xe9, 0x3c, 0xf2, 0xb1, 0xf8, 0xb0, 0xfd, 0x46, 0x02,
  0x08, 0x06, 0xd6, 0x0f, 0x0e, 0x11, 0x6a, 0x16, 0xa0, 0x1c, 0x22, 0x26,
  0x08, 0x26, 0x7d, 0x32, 0x5a, 0xcd, 0xf1, 0x8c, 0xde, 0x12, 0xd3, 0x27,
  0xb0, 0x4b, 0x15, 0x03, 0x2e, 0x92, 0x3f, 0x87, 0xf9, 0xc1, 0x1e, 0x30,
  0x62, 0x4b, 0x42, 0x21, 0xcb, 0x9b, 0x70, 0x8c, 0x1d, 0x8b, 0x25, 0xae,
  0x99, 0x41, 0xb2, 0x40, 0xa9, 0x4d, 0x32, 0x1a, 0xf5, 0xb3, 0xc2, 0x8e,
  0x5f, 0xbc, 0x43, 0x41, 0xed, 0x43, 0x57, 0x28, 0xbd, 0xc3, 0xf9, 0xae,
  0x07, 0xa1, 0x39, 0xdf, 0x27, 0x16, 0x44, 0xc6, 0x3c, 0xc6, 0x6d, 0xbc,
  0xf2, 0xc8, 0x53, 0xc5, 0x76, 0xce, 0xbf, 0xd3, 0x1b, 0xde, 0x7d, 0xe2,
  0x8f, 0xea, 0x76, 0xec, 0x7d, 0xf2, 0xfc, 0xfa, 0xa8, 0x00, 0x7b, 0x07,
  0x80, 0x0c, 0x35, 0x11, 0x2a, 0x15, 0xf1, 0x1d, 0x94, 0x20, 0xfa, 0x25,
  0x9a, 0x2a, 0xf0, 0x2d, 0x44, 0x31, 0xc4, 0x36, 0x68, 0x39, 0xc5, 0x3d,
  0x0d, 0x3e, 0xdf, 0x40, 0x49, 0x41, 0xc8, 0x44, 0x84, 0x43, 0x88, 0x47,
  0xfb, 0x42, 0x16, 0x4b, 0x31, 0x17, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x33, 0x35, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x33,
  0x35, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00,
};

const unsigned char fifteen_wav[] PROGMEM = {  
  0xa3, 0x49, 0xd2, 0x3e, 0x81, 0xf3, 0xe7, 0xfa, 0x1c, 0xfa, 0xaa, 0x06,
  0xd5, 0xf8, 0xd5, 0xfa, 0xd4, 0xfd, 0x25, 0x02, 0xf9, 0xf9, 0x5e, 0xfb,
  0x42, 0xff, 0xa6, 0xff, 0xc3, 0xfa, 0xd5, 0xfb, 0xe2, 0xff, 0x10, 0xfe,
  0x4e, 0xfb, 0x62, 0xfc, 0xfb, 0xff, 0x14, 0xfd, 0xb0, 0xfb, 0xf2, 0xfc,
  0xc7, 0xff, 0x73, 0xfc, 0x01, 0xfc, 0x80, 0xfd, 0x5a, 0xff, 0x1e, 0xfc,
  0x44, 0xfc, 0xfd, 0xfd, 0xd5, 0xfe, 0xef, 0xfb, 0x90, 0xfc, 0x57, 0xfe,
  0x4e, 0xfe, 0xda, 0xfb, 0xe8, 0xfc, 0x87, 0xfe, 0xd5, 0xfd, 0xd2, 0xfb,
  0x4b, 0xfd, 0x91, 0xfe, 0x66, 0xfd, 0xe7, 0xfb, 0xa2, 0xfd, 0x8e, 0xfe,
  0xeb, 0xfc, 0x2a, 0xfc, 0xd7, 0xfd, 0x8a, 0xfe, 0x6a, 0xfc, 0x96, 0xfc,
  0xe7, 0xfd, 0x8b, 0xfe, 0xdf, 0xfb, 0x2c, 0xfd, 0xc9, 0xfd, 0x93, 0xfe,
  0x5d, 0xfb, 0xd1, 0xfd, 0xa4, 0xfd, 0x7b, 0xfe, 0x0c, 0xfb, 0x5b, 0xfe,
  0x92, 0xfd, 0x24, 0xfe, 0x1b, 0xfb, 0x95, 0xfe, 0xca, 0xfd, 0x5c, 0xfd,
  0xc9, 0xfb, 0x27, 0xfe, 0xbd, 0xfe, 0x88, 0xfb, 0xfc, 0xfd, 0xac, 0xfb,
  0xd5, 0x02, 0x6c, 0xf3, 0x96, 0x3f, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x33, 0x36, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x33,
  0x36, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00
};

const unsigned char sixteen_wav[] PROGMEM = {
  0x06, 0xb5, 0x35, 0x89, 0x94, 0xaa, 0xad, 0xb3, 0x31, 0xeb, 0x1e, 0x01,
  0xd5, 0xee, 0xaa, 0xe9, 0x46, 0x17, 0xcb, 0x0b, 0x36, 0x0a, 0xb4, 0x06,
  0x70, 0xfb, 0x60, 0x16, 0xbd, 0x1d, 0xb3, 0x39, 0x5c, 0x40, 0xc7, 0x3c,
  0x75, 0x38, 0xbe, 0x36, 0x33, 0x46, 0xfd, 0x2f, 0xe1, 0x35, 0x1c, 0x4c,
  0x6f, 0x46, 0xb1, 0x4a, 0xdb, 0x46, 0x98, 0x49, 0x46, 0x47, 0x3e, 0x42,
  0x49, 0x41, 0x0a, 0x3e, 0x0f, 0x39, 0x7a, 0x2c, 0x29, 0x2c, 0xaa, 0x28,
  0xc1, 0x2c, 0xf2, 0x2a, 0x6b, 0x1a, 0x0d, 0x16, 0x0f, 0x0f, 0x68, 0x0c,
  0x47, 0x0e, 0xfe, 0x09, 0x22, 0x02, 0x8e, 0x02, 0x64, 0xf9, 0x4a, 0xf4,
  0xe4, 0xeb, 0xa1, 0xe9, 0xea, 0xe7, 0xaf, 0xe3, 0x9d, 0xe3, 0x2c, 0xe1,
  0x86, 0xdf, 0x9e, 0xdd, 0x92, 0xde, 0xcb, 0xdb, 0x67, 0xdd, 0x2c, 0xdb,
  0x83, 0xdd, 0x37, 0xdb, 0xc7, 0xdc, 0x30, 0xdb, 0xa6, 0xdd, 0xfe, 0xdb,
  0x4a, 0xde, 0xb2, 0xdc, 0x36, 0xe0, 0x10, 0xdf, 0x27, 0xe0, 0x61, 0xdf,
  0x38, 0xe0, 0xd1, 0xdf, 0xc7, 0xdf, 0x88, 0xe0, 0xf4, 0xdd, 0x93, 0xe0,
  0x54, 0xdb, 0x86, 0xe2, 0x9f, 0xab, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x33, 0x37, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x33,
  0x37, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00,
};

const unsigned char seventeen_wav[] PROGMEM = {
  0xe6, 0xff, 0xbd, 0xff, 0xde, 0x05, 0xe9, 0x0b, 0x4e, 0x0c, 0x93, 0x09,
  0x15, 0x03, 0xf5, 0xfa, 0x15, 0xf4, 0x05, 0xf2, 0x97, 0xf4, 0xbb, 0xfa,
  0x13, 0x03, 0x99, 0x0b, 0x54, 0x10, 0xda, 0x0e, 0x5b, 0x08, 0xc2, 0xfe,
  0x00, 0xf4, 0x54, 0xeb, 0x8c, 0xe9, 0x08, 0xf0, 0x07, 0xfc, 0xf4, 0x09,
  0xf9, 0x16, 0x2f, 0x1e, 0x1a, 0x1b, 0x20, 0x0e, 0x45, 0xfc, 0x59, 0xe9,
  0x0d, 0xd9, 0xf4, 0xd1, 0x39, 0xda, 0x56, 0xf0, 0x09, 0x0e, 0x6d, 0x2d,
  0x96, 0x46, 0xf5, 0x4b, 0x60, 0x35, 0x7c, 0x0a, 0xb3, 0xd9, 0xab, 0xab,
  0x66, 0x8b, 0x0c, 0x91, 0x52, 0xcc, 0xa6, 0x25, 0x2c, 0x69, 0x78, 0x77,
  0xcd, 0x5a, 0x29, 0x2d, 0x43, 0xfc, 0xc6, 0xcf, 0x0e, 0xb5, 0x39, 0xb6,
  0x83, 0xcd, 0xfd, 0xec, 0x5e, 0x0b, 0xff, 0x22, 0xc5, 0x2d, 0xb1, 0x28,
  0x29, 0x19, 0x21, 0x06, 0x06, 0xf4, 0xe8, 0xe5, 0x33, 0xe1, 0xe7, 0xe6,
  0xb0, 0xf3, 0x91, 0x01, 0x37, 0x0e, 0x91, 0x15, 0x59, 0x15, 0x3b, 0x0d,
  0x98, 0x02, 0x7f, 0xf8, 0x9a, 0xf1, 0x16, 0xef, 0x05, 0xf3, 0x2d, 0xfb,
  0xc8, 0x03, 0x7b, 0x0a, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00, 0x00, 0x00,
  0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00, 0x00, 0x00,
  0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00, 0x00, 0x00,
  0x32, 0x36, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00, 0x00, 0x00,
  0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x54, 0x52,
  0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x32, 0x36, 0x54,
  0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 
  };


const unsigned char eighteen_wav[] PROGMEM = {  
  0xe9, 0xff, 0x4b, 0xff, 0x41, 0x01, 0x08, 0x05, 0xe0, 0x05, 0x94, 0x03,
  0xd5, 0xff, 0xde, 0xfc, 0x11, 0xfb, 0xb5, 0xfa, 0xce, 0xfc, 0xe8, 0x00,
  0x87, 0x04, 0x56, 0x05, 0x39, 0x03, 0xfb, 0xfe, 0x17, 0xfa, 0x12, 0xf7,
  0xbf, 0xf8, 0x40, 0xff, 0xfb, 0x07, 0x69, 0x0f, 0xd5, 0x11, 0x25, 0x0b,
  0xa8, 0xfa, 0x6b, 0xe6, 0x5a, 0xd9, 0xd9, 0xdb, 0x16, 0xf0, 0xa2, 0x11,
  0xcb, 0x35, 0xda, 0x4b, 0xb0, 0x43, 0x2b, 0x18, 0x6a, 0xd6, 0x64, 0x9b,
  0xa4, 0x87, 0x2b, 0xac, 0x48, 0xfb, 0xed, 0x4c, 0x33, 0x77, 0xe4, 0x68,
  0x31, 0x30, 0x36, 0xed, 0x45, 0xbe, 0x77, 0xb2, 0x5e, 0xc6, 0x46, 0xea,
  0xf0, 0x0c, 0xd7, 0x22, 0x18, 0x27, 0x35, 0x1b, 0x09, 0x07, 0x98, 0xf5,
  0xc6, 0xed, 0x85, 0xef, 0xbb, 0xf6, 0x8c, 0xff, 0x78, 0x06, 0xb4, 0x08,
  0x10, 0x06, 0x22, 0x01, 0xbf, 0xfc, 0x5c, 0xfa, 0xde, 0xfa, 0x3c, 0xfe,
  0x77, 0x02, 0xd7, 0x04, 0xb4, 0x04, 0xf0, 0x02, 0x1f, 0x00, 0x71, 0xfc,
  0xd7, 0xf9, 0x61, 0xfa, 0xcd, 0xfd, 0x80, 0x01, 0x3e, 0x04, 0x2d, 0x05,
  0x67, 0x04, 0xc4, 0x00, 0xa2, 0xfe, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x37, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x32,
  0x37, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00, 
  };


const unsigned char nineteen_wav[] PROGMEM = {
  0x75, 0x00, 0xa9, 0xfb, 0x66, 0xf7, 0xf1, 0xfd, 0x14, 0x05, 0xb0, 0x09,
  0xc2, 0x07, 0xc7, 0xfe, 0xd3, 0xf4, 0x9a, 0xf4, 0x20, 0xff, 0x4e, 0x09,
  0x54, 0x0a, 0x06, 0x04, 0x32, 0xfc, 0xe1, 0xf5, 0xa1, 0xf5, 0x11, 0xff,
  0x6b, 0x0c, 0x3e, 0x10, 0xc5, 0x05, 0x5b, 0xf6, 0xa4, 0xee, 0xb0, 0xf3,
  0x3a, 0x03, 0x25, 0x13, 0x07, 0x15, 0x32, 0x03, 0xa4, 0xeb, 0x89, 0xe3,
  0xb3, 0xf1, 0x68, 0x0b, 0x88, 0x21, 0x70, 0x26, 0x94, 0x10, 0x8c, 0xe6,
  0xb9, 0xc6, 0xcb, 0xd1, 0xa8, 0x08, 0x85, 0x44, 0x54, 0x53, 0x3d, 0x1f,
  0x67, 0xc5, 0xb5, 0x89, 0x47, 0xa3, 0xab, 0x05, 0xe9, 0x63, 0x94, 0x74,
  0xa1, 0x32, 0x5d, 0xd9, 0x24, 0xab, 0x21, 0xc0, 0x28, 0xfe, 0x46, 0x32,
  0x8d, 0x38, 0xca, 0x15, 0x59, 0xec, 0x18, 0xd9, 0x7e, 0xe0, 0xc2, 0xf7,
  0x08, 0x11, 0x3c, 0x1d, 0x0b, 0x13, 0x02, 0xfb, 0x9b, 0xea, 0x5d, 0xee,
  0xf9, 0xfe, 0xe7, 0x0d, 0x9e, 0x11, 0xd7, 0x08, 0x3e, 0xf9, 0xf5, 0xef,
  0xfb, 0xf4, 0xbd, 0x02, 0x43, 0x0b, 0x4d, 0x0a, 0x61, 0x03, 0xd1, 0xfb,
  0xb5, 0xf5, 0x35, 0xf8, 0x6c, 0x00, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x38, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x32,
  0x38, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00, 
  };

const unsigned char twenty_wav[] PROGMEM = {
  0x0d, 0xb3, 0x2a, 0x9d, 0x42, 0xd2, 0x09, 0xd3, 0x99, 0xe8, 0x4b, 0xed,
  0xa8, 0xfc, 0x36, 0x01, 0xcf, 0x0b, 0x06, 0x0f, 0x6b, 0x15, 0xf1, 0x16,
  0xaf, 0x19, 0x88, 0x19, 0x44, 0x19, 0xb5, 0x17, 0x39, 0x15, 0xb3, 0x12,
  0xcf, 0x0e, 0xe1, 0x0b, 0x61, 0x07, 0x94, 0x04, 0x36, 0x00, 0xfa, 0xfd,
  0x5f, 0xfa, 0x06, 0xf9, 0x8b, 0xf6, 0x49, 0xf6, 0x17, 0xf5, 0xf5, 0xf5,
  0x01, 0xf6, 0xd7, 0xf7, 0xeb, 0xf8, 0x6e, 0xfb, 0x36, 0xfd, 0x07, 0x00,
  0x15, 0x02, 0xd8, 0x04, 0xbe, 0x06, 0x11, 0x09, 0x77, 0x0a, 0x12, 0x0c,
  0xb7, 0x0c, 0x6b, 0x0d, 0x36, 0x0d, 0xfe, 0x0c, 0xf5, 0x0b, 0xe0, 0x0a,
  0x3e, 0x09, 0x79, 0x07, 0x94, 0x05, 0x5d, 0x03, 0x93, 0x01, 0x35, 0xff,
  0xed, 0xfd, 0xaa, 0xfb, 0x38, 0xfb, 0x4c, 0xf9, 0xe7, 0xf9, 0x78, 0xf8,
  0x39, 0xfa, 0x45, 0xf9, 0x1d, 0xfc, 0x9b, 0xfb, 0x47, 0xff, 0x1b, 0xff,
  0x34, 0x03, 0x34, 0x03, 0x3a, 0x07, 0x48, 0x07, 0xa2, 0x0a, 0xb7, 0x0a,
  0xbf, 0x0c, 0x0b, 0x0d, 0x00, 0x0d, 0x16, 0x0e, 0xe7, 0x0a, 0x4c, 0x0e,
  0x47, 0x05, 0xff, 0x10, 0xc5, 0xc1, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x39, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x32,
  0x39, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00, 
  };

const unsigned char twentyOne_wav[] PROGMEM = {
  0x0c, 0xb2, 0x00, 0x80, 0xa3, 0xa6, 0x84, 0xb5, 0xee, 0xd2, 0xdf, 0xe1,
  0x98, 0xf7, 0x9d, 0x02, 0x68, 0x10, 0x06, 0x16, 0x36, 0x1c, 0x95, 0x1c,
  0x8a, 0x1c, 0x08, 0x19, 0xf2, 0x14, 0x9c, 0x0f, 0xf0, 0x09, 0xe5, 0x04,
  0xd6, 0xff, 0xba, 0xfc, 0xb5, 0xf9, 0x4d, 0xf9, 0xd9, 0xf8, 0xf7, 0xfa,
  0xa1, 0xfc, 0x5b, 0x00, 0x12, 0x03, 0x1d, 0x07, 0x9d, 0x09, 0xc0, 0x0c,
  0x01, 0x0e, 0x6e, 0x0f, 0xf9, 0x0e, 0x8b, 0x0e, 0x89, 0x0c, 0xb7, 0x0a,
  0xd9, 0x07, 0x90, 0x05, 0xc0, 0x02, 0xf9, 0x00, 0x17, 0xff, 0x86, 0xfe,
  0x11, 0xfe, 0xf0, 0xfe, 0xe7, 0xff, 0xe1, 0x01, 0xcc, 0x03, 0x30, 0x06,
  0x47, 0x08, 0x39, 0x0a, 0xbc, 0x0b, 0x80, 0x0c, 0x04, 0x0d, 0x40, 0x0c,
  0xbd, 0x0b, 0x93, 0x09, 0x78, 0x08, 0x72, 0x05, 0x82, 0x04, 0x57, 0x01,
  0x53, 0x01, 0xb8, 0xfe, 0x1c, 0x00, 0x80, 0xfe, 0x55, 0x01, 0xbb, 0x00,
  0x94, 0x04, 0x99, 0x04, 0xa4, 0x08, 0xc0, 0x08, 0xf7, 0x0b, 0xbf, 0x0b,
  0x31, 0x0d, 0xa0, 0x0c, 0x93, 0x0b, 0x5b, 0x0b, 0x15, 0x07, 0x34, 0x09,
  0xb0, 0xff, 0xa2, 0x0a, 0xd1, 0xbe, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x33, 0x30, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x33,
  0x30, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00,
  };

const unsigned char twentyTwo_wav[] PROGMEM = { 
  0xc8, 0xd5, 0x67, 0xe7, 0xf4, 0x15, 0xf4, 0x0c, 0x24, 0x15, 0x69, 0x11,
  0x6b, 0x16, 0x69, 0x14, 0xa9, 0x17, 0x78, 0x16, 0x60, 0x18, 0x9a, 0x17,
  0x61, 0x18, 0xcb, 0x17, 0xa0, 0x17, 0x12, 0x17, 0x23, 0x16, 0x87, 0x15,
  0x03, 0x14, 0x53, 0x13, 0x6c, 0x11, 0xad, 0x10, 0x94, 0x0e, 0xd1, 0x0d,
  0xbf, 0x0b, 0x02, 0x0b, 0x2b, 0x09, 0x8d, 0x08, 0x13, 0x07, 0xb1, 0x06,
  0xae, 0x05, 0xa1, 0x05, 0x2b, 0x05, 0x7d, 0x05, 0xa4, 0x05, 0x59, 0x06,
  0x19, 0x07, 0x35, 0x08, 0x78, 0x09, 0xf6, 0x0a, 0x9f, 0x0c, 0x68, 0x0e,
  0x55, 0x10, 0x47, 0x12, 0x49, 0x14, 0x3f, 0x16, 0x22, 0x18, 0xf0, 0x19,
  0x7f, 0x1b, 0xf1, 0x1c, 0xfa, 0x1d, 0xe0, 0x1e, 0x2f, 0x1f, 0x56, 0x1f,
  0xcd, 0x1e, 0xfe, 0x1d, 0x8d, 0x1c, 0x9d, 0x1a, 0x2e, 0x18, 0x0f, 0x15,
  0x9d, 0x11, 0x3f, 0x0d, 0xd6, 0x08, 0x3c, 0x03, 0xf7, 0xfd, 0x31, 0xf7,
  0x34, 0xf1, 0x60, 0xe9, 0xd9, 0xe2, 0x2b, 0xda, 0x4a, 0xd3, 0x02, 0xca,
  0xfb, 0xc2, 0x5c, 0xb9, 0x6d, 0xb2, 0xbb, 0xa8, 0x27, 0xa2, 0x93, 0x98,
  0xce, 0x92, 0xc5, 0x88, 0x8a, 0x9c, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x33, 0x31, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x33,
  0x31, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00, 
  };
  
const unsigned char twentyThree_wav[] PROGMEM = {
  0x84, 0xb2, 0x00, 0x80, 0x73, 0x93, 0x3e, 0x90, 0xb6, 0x9f, 0x62, 0xa3,
  0x1c, 0xb2, 0x4f, 0xb9, 0x94, 0xc7, 0x9d, 0xd0, 0xd3, 0xdd, 0xc2, 0xe7,
  0x1d, 0xf3, 0x6d, 0xfc, 0x17, 0x06, 0x8d, 0x0e, 0x36, 0x16, 0x6c, 0x1d,
  0x27, 0x23, 0x07, 0x29, 0x6b, 0x2c, 0xaa, 0x30, 0xb6, 0x32, 0x70, 0x35,
  0x38, 0x36, 0xc4, 0x37, 0x85, 0x37, 0x23, 0x38, 0x29, 0x37, 0xfb, 0x36,
  0x8d, 0x35, 0xd7, 0x34, 0x13, 0x33, 0xf3, 0x31, 0x01, 0x30, 0xa5, 0x2e,
  0x9d, 0x2c, 0x20, 0x2b, 0x0f, 0x29, 0x7f, 0x27, 0x82, 0x25, 0xed, 0x23,
  0xfa, 0x21, 0x70, 0x20, 0x7e, 0x1e, 0x0f, 0x1d, 0x26, 0x1b, 0xba, 0x19,
  0x03, 0x18, 0x91, 0x16, 0xf1, 0x14, 0x6c, 0x13, 0x01, 0x12, 0x52, 0x10,
  0x05, 0x0f, 0x24, 0x0d, 0x19, 0x0c, 0xe4, 0x09, 0xfb, 0x08, 0x64, 0x06,
  0x92, 0x05, 0x7a, 0x02, 0xba, 0x01, 0x1d, 0xfe, 0x41, 0xfd, 0x08, 0xf9,
  0xf7, 0xf7, 0x1e, 0xf3, 0x9d, 0xf1, 0x2e, 0xec, 0x02, 0xea, 0x23, 0xe4,
  0xf2, 0xe0, 0xd2, 0xda, 0x5b, 0xd6, 0x36, 0xd0, 0x1d, 0xca, 0xb2, 0xc5,
  0x7a, 0xbb, 0xb3, 0xba, 0xdc, 0x96, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x33, 0x33, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x33,
  0x33, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00,
  };

const unsigned char twentyFour_wav[] PROGMEM = {
  0x6a, 0xb2, 0x01, 0x80, 0x45, 0x92, 0xec, 0x8c, 0x82, 0x98, 0x5d, 0x97,
  0x58, 0xa0, 0x14, 0xa1, 0x73, 0xa8, 0x78, 0xaa, 0xb1, 0xb0, 0x9e, 0xb3,
  0xe9, 0xb8, 0x79, 0xbc, 0x28, 0xc1, 0x1b, 0xc5, 0x39, 0xc9, 0x90, 0xcd,
  0x4e, 0xd1, 0xc0, 0xd5, 0x26, 0xd9, 0xac, 0xdd, 0xc6, 0xe0, 0x45, 0xe5,
  0x5c, 0xe8, 0xbb, 0xec, 0x9e, 0xef, 0xcc, 0xf3, 0xaa, 0xf6, 0xa2, 0xfa,
  0x42, 0xfd, 0x18, 0x01, 0xbe, 0x03, 0x4a, 0x07, 0xd0, 0x09, 0x1e, 0x0d,
  0x84, 0x0f, 0x99, 0x12, 0xcd, 0x14, 0xb0, 0x17, 0xd0, 0x19, 0x6d, 0x1c,
  0x5e, 0x1e, 0xcf, 0x20, 0x8a, 0x22, 0xc9, 0x24, 0x59, 0x26, 0x3f, 0x28,
  0xad, 0x29, 0x50, 0x2b, 0xb2, 0x2c, 0xdd, 0x2d, 0x45, 0x2f, 0x05, 0x30,
  0x64, 0x31, 0xaf, 0x31, 0x20, 0x33, 0xaa, 0x32, 0xd3, 0x34, 0x29, 0x33,
  0xe3, 0x34, 0x07, 0x33, 0x10, 0x35, 0xeb, 0x32, 0x0f, 0x35, 0xdd, 0x32,
  0x1a, 0x35, 0xee, 0x31, 0x7b, 0x33, 0xe8, 0x30, 0x87, 0x31, 0x6a, 0x2f,
  0xfb, 0x2e, 0xad, 0x2d, 0xae, 0x2b, 0x01, 0x2c, 0x33, 0x27, 0xfb, 0x2a,
  0x51, 0x20, 0x4d, 0x2e, 0xc3, 0xcf, 0x4c, 0x49, 0x53, 0x54, 0x1c, 0x00,
  0x00, 0x00, 0x49, 0x4e, 0x46, 0x4f, 0x49, 0x4e, 0x41, 0x4d, 0x04, 0x00,
  0x00, 0x00, 0x32, 0x34, 0x72, 0x00, 0x49, 0x54, 0x52, 0x4b, 0x04, 0x00,
  0x00, 0x00, 0x33, 0x32, 0x00, 0x00, 0x69, 0x64, 0x33, 0x20, 0x26, 0x00,
  0x00, 0x00, 0x49, 0x44, 0x33, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b,
  0x54, 0x52, 0x43, 0x4b, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x33,
  0x32, 0x54, 0x49, 0x54, 0x32, 0x00, 0x00, 0x00, 0x04, 0x00,
  };

unsigned int wav_len = 238;
int p = 0;

int scale = 0; 

int pot1Raw = 0;
int pitch = 0;

int pot2Raw = 0;
int fuckery = 0;

int pot3Raw = 0;
int select = 0;

int trigState = 0;

int yaAsshole = 0;

char sample = 0;
char sample1 = 0; 

void setup() {

    clock_prescale_set(clock_div_2); //NO PROCESSOR PRESCALE

  // Enable 64 MHz PLL and use as source for Timer1
  PLLCSR = 1 << PCKE | 1 << PLLE;

  // Set up Timer/Counter1 for PWM output
  TIMSK = 0;                              // Timer interrupts OFF
  TCCR1 = 1 << PWM1A | 2 << COM1A0 | 1 << CS10; // PWM A, clear on match, 1:1 prescale

  // Set up Timer/Counter0 for 8kHz interrupt to output samples.
  TCCR0A = 3 << WGM00;                    // Fast PWM
  TCCR0B = 1 << WGM02 | 2 << CS00;        // 1/8 prescale
  TIMSK = 1 << OCIE0A;                    // Enable compare match

  //pinMode(0, INPUT);
  pinMode(1, OUTPUT);
}

void loop() {

  //trigState = digitalRead(0); // Read digital pin 0 as input
  pot1Raw = analogRead(A1); // Read Pin 1 as analog input
  pot2Raw = analogRead(A3); // Read pin 2 as analog input
  pot3Raw = analogRead(A2); // Read pin 3 as analog input
  fuckery = map(pot1Raw, 1, 1023, 0, 255); 
  
  select = map(pot3Raw, 1, 1023, 1, 23);
  
  pitch = map(pot2Raw, 1, 1023, 200, 30);

  if (trigState == HIGH){
  yaAsshole = random(-8, 8);
  }

  else {
  yaAsshole = 1; 
  }

  OCR0A = pitch;
  
}

// Sample interrupt 
ISR(TIMER0_COMPA_vect) {
   
    if (p >= wav_len)
    {
    p = 0;
    }   

    switch (select)
    {
    case 1:
      sample = pgm_read_byte(&one_wav[p++]);
      sample1 = pgm_read_byte(&four_wav[p++]);
      break;

    case 2:
      sample = pgm_read_byte(&two_wav[p++]);
      break;

    case 3:
      sample = pgm_read_byte(&three_wav[p++]);
      break;

    case 4:
      sample = pgm_read_byte(&four_wav[p++]);
      break;
      
    case 5:
      sample = pgm_read_byte(&five_wav[p++]);
      break;

    case 6:
      sample = pgm_read_byte(&six_wav[p++]);
      break;

    case 7:
      sample = pgm_read_byte(&seven_wav[p++]);
      break;

    case 8:
      sample = pgm_read_byte(&eight_wav[p++]);
      break;

    case 9:
      sample = pgm_read_byte(&nine_wav[p++]);
      break;

    case 10:
      sample = pgm_read_byte(&ten_wav[p++]);
      break;
      
    case 11:
      sample = pgm_read_byte(&eleven_wav[p++]);
      break;

    case 12:
      sample = pgm_read_byte(&twelve_wav[p++]);
      break;

    case 13:
      sample = pgm_read_byte(&thirteen_wav[p++]);
      break;

    case 14:
      sample = pgm_read_byte(&fourteen_wav[p++]);
      break;
      
    case 15:
      sample = pgm_read_byte(&fifteen_wav[p++]);
      break;

    case 16:
      sample = pgm_read_byte(&sixteen_wav[p++]);
      break;

    case 17:
      sample = pgm_read_byte(&seventeen_wav[p++]);
      break;

    case 18:
      sample = pgm_read_byte(&eighteen_wav[p++]);
      break;

    case 19:
      sample = pgm_read_byte(&nineteen_wav[p++]);
      break;


      } 
      
  OCR1A = (sample + fuckery) * yaAsshole; 

}

#ifndef UNMUNGER_H
#define UNMUNGER_H

#include <stdint.h>

void unmunge_frame(unsigned char* data, int size, int count, int stream_type, unsigned int key1, unsigned int key2);

#endif // UNMUNGER_H

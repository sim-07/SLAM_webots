#ifndef COMMON_H
#define COMMON_H

enum MessType {
    INFO = 0,
    ERROR = 1
};

struct Message {
    MessType type;
    char mess[64];
};

#endif
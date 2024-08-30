//
// Created by grilo on 20/07/23.
//

#include "utils.h"

uint64_t getCurrentEpochMicroseconds()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                                                                        now().time_since_epoch()).count();
    return us;
}

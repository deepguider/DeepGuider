#ifndef __TEST_UTILITY__
#define __TEST_UTILITY__

#include "utils/utility.hpp"
#include "utils/vvs.h"

int testUtilRingBuffer()
{
    dg::RingBuffer<int> buffer;
    VVS_CHECK_TRUE(buffer.empty());
    VVS_CHECK_TRUE(buffer.data_count() == 0);
    VVS_CHECK_TRUE(buffer.buffer_size() == 0);

    buffer.resize(5);
    VVS_CHECK_TRUE(buffer.empty());
    VVS_CHECK_TRUE(buffer.data_count() == 0);
    VVS_CHECK_TRUE(buffer.buffer_size() == 5);

    buffer.push_back(1);
    buffer.push_back(2);
    VVS_CHECK_TRUE(buffer.data_count() == 2);
    VVS_CHECK_TRUE(buffer[0] == 1);
    VVS_CHECK_TRUE(buffer[1] == 2);
    VVS_CHECK_TRUE(buffer.front() == 1);
    VVS_CHECK_TRUE(buffer.back() == 2);

    buffer.push_back(3);
    buffer.push_back(4);
    buffer.push_back(5);
    buffer.push_back(6);
    buffer.push_back(7);
    buffer.push_back(8);
    VVS_CHECK_TRUE(buffer.data_count() == 5);
    VVS_CHECK_TRUE(buffer[0] == 4);
    VVS_CHECK_TRUE(buffer[1] == 5);
    VVS_CHECK_TRUE(buffer.front() == 4);
    VVS_CHECK_TRUE(buffer.back() == 8);

    VVS_CHECK_TRUE(buffer.insert(2, 9) == 1);
    VVS_CHECK_TRUE(buffer.data_count() == 5);
    VVS_CHECK_TRUE(buffer[0] == 5);
    VVS_CHECK_TRUE(buffer[1] == 9);
    VVS_CHECK_TRUE(buffer[2] == 6);
    VVS_CHECK_TRUE(buffer.front() == 5);
    VVS_CHECK_TRUE(buffer.back() == 8);

    VVS_CHECK_TRUE(buffer.erase(3, 10) == false);
    VVS_CHECK_TRUE(buffer.erase(-1, 2) == false);
    VVS_CHECK_TRUE(buffer.erase(3, -1) == true);
    VVS_CHECK_TRUE(buffer.erase(0, 0) == true);
    VVS_CHECK_TRUE(buffer[0] == 9);
    VVS_CHECK_TRUE(buffer[1] == 6);
    VVS_CHECK_TRUE(buffer.data_count() == 2);
    VVS_CHECK_TRUE(buffer.front() == 9);
    VVS_CHECK_TRUE(buffer.back() == 6);

    return 0;
}

#endif // End of '__TEST_UTILITY__'

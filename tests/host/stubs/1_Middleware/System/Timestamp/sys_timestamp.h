#ifndef TESTS_HOST_STUBS_SYS_TIMESTAMP_H
#define TESTS_HOST_STUBS_SYS_TIMESTAMP_H

#include "tim.h"
#include <cstdint>

extern int fake_sequence;
extern int fake_timestamp_init_sequence;
extern int fake_timestamp_overflow_count;
extern TIM_HandleTypeDef *fake_timestamp_handler;
extern std::uint64_t fake_timestamp_now;

class Class_Timestamp
{
public:
    void Init(TIM_HandleTypeDef *htim)
    {
        fake_timestamp_handler = htim;
        fake_timestamp_init_sequence = ++fake_sequence;
    }

    void TIM_3600s_PeriodElapsedCallback()
    {
        ++fake_timestamp_overflow_count;
    }

    std::uint64_t Get_Current_Timestamp() const
    {
        return fake_timestamp_now;
    }
};

extern Class_Timestamp SYS_Timestamp;

#endif

#ifndef TESTS_HOST_MOTOR_STUBS_ALG_BASIC_H
#define TESTS_HOST_MOTOR_STUBS_ALG_BASIC_H

#include <cstdint>
#include <cstring>

#define BASIC_MATH_CELSIUS_TO_KELVIN 273.15f
#define BASIC_MATH_RPM_TO_RADPS 0.104719755f
#define PI 3.14159265358979323846f

template <typename Type>
Type Basic_Math_Constrain(Type *value, const Type min, const Type max)
{
    if (*value < min)
    {
        *value = min;
    }
    else if (*value > max)
    {
        *value = max;
    }
    return *value;
}

inline std::uint16_t Basic_Math_Endian_Reverse_16(void *source, void *destination)
{
    std::uint16_t source_value = 0U;
    std::memcpy(&source_value, source, sizeof(source_value));
    const std::uint16_t result = static_cast<std::uint16_t>((source_value >> 8U) | (source_value << 8U));
    if (destination != nullptr)
    {
        std::memcpy(destination, &result, sizeof(result));
    }
    return result;
}

inline std::int32_t Basic_Math_Float_To_Int(float value, float value_min, float value_max,
                                            std::int32_t integer_min, std::int32_t integer_max)
{
    const float normalized = (value - value_min) / (value_max - value_min);
    return static_cast<std::int32_t>(normalized * static_cast<float>(integer_max - integer_min) + static_cast<float>(integer_min));
}

inline float Basic_Math_Int_To_Float(std::int32_t value, std::int32_t integer_min,
                                     std::int32_t integer_max, float value_min, float value_max)
{
    return static_cast<float>(value - integer_min) / static_cast<float>(integer_max - integer_min) * (value_max - value_min) + value_min;
}

inline float Basic_Math_Modulus_Normalization(const float value, const float)
{
    return value;
}

#endif

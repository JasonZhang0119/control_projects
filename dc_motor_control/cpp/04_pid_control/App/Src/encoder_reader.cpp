#include "encoder_reader.hpp"

#include "stm32f1xx_hal_tim.h"

#include <cstdint>


namespace
{
static constexpr float kTwoPi = 6.2831853071795864769F;
}


EncoderReader::EncoderReader(
    TIM_HandleTypeDef* htim,
    const float counts_per_revolution
)
    : htim_(htim),
      last_count_(0U),
      counts_per_revolution_(counts_per_revolution)
{
}


void EncoderReader::Start()
{
    HAL_TIM_Encoder_Start(this->htim_, TIM_CHANNEL_ALL);
}


void EncoderReader::Reset()
{
    __HAL_TIM_SET_COUNTER(this->htim_, 0U);
    this->last_count_ = 0U;
}


uint16_t EncoderReader::GetCount() const
{
    return static_cast<uint16_t>(
        __HAL_TIM_GET_COUNTER(this->htim_)
    );
}


int16_t EncoderReader::GetDelta()
{
    const uint16_t current_count = this->GetCount();

    const int16_t delta_count =
        static_cast<int16_t>(current_count - this->last_count_);

    this->last_count_ = current_count;

    return delta_count;
}


EncoderMeasurement EncoderReader::Sample(const float sample_time_s)
{
    EncoderMeasurement measurement{};

    measurement.count = this->GetCount();

    measurement.delta_count =
        static_cast<int16_t>(measurement.count - this->last_count_);

    this->last_count_ = measurement.count;

    measurement.counts_per_second =
        static_cast<float>(measurement.delta_count) / sample_time_s;

    const float revolutions_per_second =
        measurement.counts_per_second / this->counts_per_revolution_;

    measurement.rpm =
        revolutions_per_second * 60.0F;

    measurement.rad_per_second =
        revolutions_per_second * kTwoPi;

    return measurement;
}
#include "encoder_reader.hpp"

#include "stm32f1xx_hal_tim.h"

#include <cstdint>


namespace
{

/**
 * @brief 2*pi 常数。
 */
static constexpr float kTwoPi = 6.2831853071795864769F;


/**
 * @brief 防止除零的小正数。
 */
static constexpr float kMinPositiveSampleTime = 1.0e-6F;

}  // namespace


/**
 * @brief 构造编码器读取器。
 */
EncoderReader::EncoderReader(
    TIM_HandleTypeDef* htim,
    const float counts_per_revolution
)
    : htim_(htim),
      last_count_(0U),
      position_count_(0),
      counts_per_revolution_(counts_per_revolution)
{
}


/**
 * @brief 启动 TIM Encoder Mode。
 */
void EncoderReader::Start()
{
    HAL_TIM_Encoder_Start(this->htim_, TIM_CHANNEL_ALL);
}


/**
 * @brief 停止 TIM Encoder Mode。
 */
void EncoderReader::Stop()
{
    HAL_TIM_Encoder_Stop(this->htim_, TIM_CHANNEL_ALL);
}


/**
 * @brief 清零硬件 counter 和软件连续位置。
 */
void EncoderReader::Reset()
{
    this->Reset(0);
}


/**
 * @brief 清零硬件 counter，并设置软件连续位置。
 */
void EncoderReader::Reset(const int32_t position_count)
{
    __HAL_TIM_SET_COUNTER(this->htim_, 0U);

    this->last_count_ = 0U;
    this->position_count_ = position_count;
}


/**
 * @brief 读取当前 TIMx counter。
 */
uint16_t EncoderReader::GetCount() const
{
    return static_cast<uint16_t>(
        __HAL_TIM_GET_COUNTER(this->htim_)
    );
}


/**
 * @brief 获取当前软件连续位置。
 */
int32_t EncoderReader::GetPositionCount() const
{
    return this->position_count_;
}


/**
 * @brief 手动设置当前软件连续位置。
 */
void EncoderReader::SetPositionCount(const int32_t position_count)
{
    this->position_count_ = position_count;
}


/**
 * @brief 获取当前位置，单位 revolution。
 */
float EncoderReader::GetPositionRevolution() const
{
    return static_cast<float>(this->position_count_) /
           this->counts_per_revolution_;
}


/**
 * @brief 获取当前位置，单位 rad。
 */
float EncoderReader::GetPositionRad() const
{
    return this->GetPositionRevolution() * kTwoPi;
}


/**
 * @brief 读取当前 encoder 增量。
 */
int16_t EncoderReader::GetDelta()
{
    const uint16_t current_count = this->GetCount();

    const int16_t delta_count =
        static_cast<int16_t>(current_count - this->last_count_);

    this->last_count_ = current_count;

    this->position_count_ += static_cast<int32_t>(delta_count);

    return delta_count;
}


/**
 * @brief 采样编码器并计算位置与速度。
 */
EncoderMeasurement EncoderReader::Sample(const float sample_time_s)
{
    EncoderMeasurement measurement{};

    const float safe_sample_time_s =
        (sample_time_s > kMinPositiveSampleTime)
            ? sample_time_s
            : kMinPositiveSampleTime;

    measurement.count = this->GetCount();

    measurement.delta_count =
        static_cast<int16_t>(measurement.count - this->last_count_);

    this->last_count_ = measurement.count;

    this->position_count_ +=
        static_cast<int32_t>(measurement.delta_count);

    measurement.position_count =
        this->position_count_;

    measurement.position_revolution =
        static_cast<float>(measurement.position_count) /
        this->counts_per_revolution_;

    measurement.position_rad =
        measurement.position_revolution * kTwoPi;

    measurement.counts_per_second =
        static_cast<float>(measurement.delta_count) /
        safe_sample_time_s;

    const float revolutions_per_second =
        measurement.counts_per_second /
        this->counts_per_revolution_;

    measurement.rpm =
        revolutions_per_second * 60.0F;

    measurement.rad_per_second =
        revolutions_per_second * kTwoPi;

    return measurement;
}
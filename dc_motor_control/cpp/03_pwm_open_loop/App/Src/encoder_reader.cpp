

#include "encoder_reader.hpp"
#include "stm32f1xx_hal_tim.h"
#include <cstdint>


EncoderReader::EncoderReader(TIM_HandleTypeDef* htim){

    this->htim_ = htim;

}



void EncoderReader::Start(){

    HAL_TIM_Encoder_Start(this->htim_, TIM_CHANNEL_ALL);

}

void EncoderReader::Reset(){

    __HAL_TIM_SET_COUNTER(this->htim_, 0U);

}

uint16_t EncoderReader::GetCount() const{

    return __HAL_TIM_GET_COUNTER(this->htim_);

}

int16_t EncoderReader::GetDelta(){

    const uint16_t current_count = this->GetCount();

    const int16_t delta =
        static_cast<int16_t>(current_count - this->last_count_);

    this->last_count_ = current_count;

    return delta;

}
#include "trajectory_generator.hpp"
#include <cstdint>


FixedLengthPositionTrajectory::FixedLengthPositionTrajectory(uint32_t point_count)
    : point_count_(point_count), index_(0), start_position_rad_(0), target_position_rad_(0){

}

void FixedLengthPositionTrajectory::Reset(){
    this->Set(0, 0);
}

void FixedLengthPositionTrajectory::Set(float start_position_rad, float target_position_rad){
    this->index_ = 0;
    this->start_position_rad_ = start_position_rad;
    this->target_position_rad_ = target_position_rad;
}

float FixedLengthPositionTrajectory::GetStartPosition(){
    return this->start_position_rad_;
}

float FixedLengthPositionTrajectory::GetTargetPosition(){
    return this->target_position_rad_;
}

bool FixedLengthPositionTrajectory::HasNext() const{
    if (this->index_ == this->point_count_ - 1){
        return false;
    }else{
        return true;
    }
}

float FixedLengthPositionTrajectory::Next(){
    float position;
    if (this->HasNext()){
        position = start_position_rad_;
        position += (this->target_position_rad_ - this->start_position_rad_)/this->point_count_ * this->index_;
        this->index_++;
    }else{
        position = this->target_position_rad_;
    }
    return position;
}
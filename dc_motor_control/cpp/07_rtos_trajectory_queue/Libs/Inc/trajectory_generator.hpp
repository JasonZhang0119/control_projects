


class FixedLengthPositionTrajectory {
public:
    explicit FixedLengthPositionTrajectory(uint32_t point_count);

    void Reset();
    void Set(float start_position_rad, float target_position_rad);
    float GetStartPosition();
    float GetTargetPosition();

    bool HasNext() const;

    float Next();

private:
    uint32_t point_count_;
    uint32_t index_;

    float start_position_rad_;
    float target_position_rad_;
};


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

bool FixedLengthPositionTrajectory::HasNext(){
    if (this->index_ == this->point_count_ - 1){
        return false;
    }else{
        return true;
    }
}

float FixedLengthPositionTrajectory::Next(){
    if (this->HasNext()){
        float position = start_position_rad_;
        position += (this->target_position_rad_ - this->start_position_rad_)/this->point_count_ * this->index_;
        this->index_++;
    }else{
        float position = this->target_position_rad_;
    }
    float position = start_position_rad_;
}
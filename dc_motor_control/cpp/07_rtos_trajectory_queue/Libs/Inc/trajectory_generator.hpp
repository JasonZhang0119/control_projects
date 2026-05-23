#pragma once

#include <cstdint>

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


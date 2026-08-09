
#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "typing.hpp"
#include "output_feedback_controller.hpp"

/**
 * @brief 阶梯信号发生器。
 *
 * Parameters
 * ----------
 * DataType : typename
 *     标量类型，通常为 float。
 * Ny : int
 *     输出反馈维度，为了复用控制模块接口而保留。
 * Nu : int
 *     信号输出维度。
 * NStair : int
 *     每个输出通道的阶梯点数量。
 *
 * Notes
 * -----
 * 每个通道都有一列时间戳和一列目标值。Step() 根据当前内部时间，
 * 选择最近生效的阶梯值作为输出，并最终经过 u_min/u_max 限幅。
 */
template<typename DataType, int Ny, int Nu, int NStair>
class StairSignalGenerator:
    public OutputFeedbackControllerModule<DataType, Ny, Nu>{

    public:

        using Base = OutputFeedbackControllerModule<DataType, Ny, Nu>;
        using YVector = typename Base::YVector;
        using UVector = typename Base::UVector;

        using StairTime = Eigen::Matrix<DataType, NStair, Nu>;
        using StairValue = Eigen::Matrix<DataType, NStair, Nu>;

        /**
         * @brief 设置阶梯信号的时间戳和值表。
         */
        void SetStairInfo(const StairTime& timestamps, const StairValue& values){
            this->timestamps_ = timestamps;
            this->values_ = values;
        };

        /**
         * @brief 设置采样周期。
         */
        void SetSamplingTime(const DataType& Ts){
            this->Ts_ = Ts;
        }

        /**
         * @brief 获取阶梯时间戳矩阵。
         */
        const StairTime& GetStairTime() const{
            return this->timestamps_;
        };

        /**
         * @brief 获取阶梯输出值矩阵。
         */
        const StairValue& GetStairValue() const{
            return this->values_;
        };

        /**
         * @brief 获取发生器当前内部时间。
         */
        const DataType& GetCurrentTime() const{
            return this->current_time_;
        };

        /**
         * @brief 生成一步阶梯信号输出。
         */
        UVector Step(const Eigen::Ref<const YVector>& y_ref,
                     const Eigen::Ref<const YVector>& y,
                     const Eigen::Ref<const UVector>& u_min,
                     const Eigen::Ref<const UVector>& u_max) override;
        
        /**
         * @brief 根据当前时间寻找某一通道应使用的阶梯索引。
         */
        int CheckTimeIndex(Eigen::Ref<const Eigen::Matrix<DataType, NStair, 1>> column_data, 
                           int last_index);

        /**
         * @brief 重置内部时间和当前阶梯索引。
         */
        void Reset() override{
            this->current_time_ = 0;
            this->current_index_.setZero();
        };

        /**
         * @brief 初始化发生器。
         */
        void Initialize() override{
            this->Reset();
        }

        /**
         * @brief 终止发生器并清空内部状态。
         */
        void Terminate() override{
            this->Reset();
        }

    private:
        
        StairTime timestamps_{StairTime::Zero()};
        StairValue values_{StairValue::Zero()};
        DataType Ts_{static_cast<DataType>(0.1)};
        DataType current_time_{static_cast<DataType>(0)};

        Eigen::Matrix<int, Nu, 1> current_index_{Eigen::Matrix<int, Nu, 1>::Zero()};

};


/**
 * @brief 查找当前时间对应的阶梯段索引。
 */
template<typename DataType, int Ny, int Nu, int NStair>
int StairSignalGenerator<DataType, Ny, Nu, NStair>::CheckTimeIndex(
    Eigen::Ref<const Eigen::Matrix<DataType, NStair, 1>> column_data, int last_index){

    if (this->current_time_ >= column_data(NStair - 1)){
        return NStair - 1;
    }else if(this->current_time_ < column_data(0)){
        return 0;
    }else{
        while (last_index < (NStair - 1) && column_data(last_index + 1) <= this->current_time_) {
                last_index++;
        }
    }
    return last_index;
}


/**
 * @brief 输出当前采样时刻的阶梯信号。
 */
template<typename DataType, int Ny, int Nu, int NStair>
typename StairSignalGenerator<DataType, Ny, Nu, NStair>::UVector
StairSignalGenerator<DataType, Ny, Nu, NStair>::Step(
    const Eigen::Ref<const YVector>& y_ref,
    const Eigen::Ref<const YVector>& y,
    const Eigen::Ref<const UVector>& u_min,
    const Eigen::Ref<const UVector>& u_max) {


    (void) y_ref;
    (void) y;

    // 1. 计算当前时间
    this->current_time_ += this->Ts_;
    
    // 2. 寻找对应位置并计算输出量
    UVector u_unsat = UVector::Zero();
    for (int i = 0; i < Nu; i++){
        this->current_index_(i, 0) = this->CheckTimeIndex(this->timestamps_.col(i), 
                                                          this->current_index_(i, 0));
        u_unsat(i, 0) = this->values_(this->current_index_(i, 0), i);
    }


    // 3. 控制量限幅 (Saturation)
    // 使用 Eigen 的 cwiseMin/cwiseMax 进行元素级限幅
    UVector u_sat = u_unsat.cwiseMax(u_min).cwiseMin(u_max);

    // 4. 返回实际执行的控制量
    return u_sat;
};



/**
 * @brief 线性扫频正弦信号发生器。
 *
 * Parameters
 * ----------
 * DataType : typename
 *     标量类型，通常为 float。
 * Ny : int
 *     输出反馈维度，为了复用控制模块接口而保留。
 * Nu : int
 *     信号输出维度。
 *
 * Notes
 * -----
 * 每个输出通道可以独立配置起止时间、起止频率、偏置和幅值。
 * Step() 会按当前时间计算瞬时频率并积分相位，最终输出扫频正弦信号。
 */
template<typename DataType, int Ny, int Nu>
class ChirpSignalGenerator:
    public OutputFeedbackControllerModule<DataType, Ny, Nu>{

    public:

        using Base = OutputFeedbackControllerModule<DataType, Ny, Nu>;
        using YVector = typename Base::YVector;
        using UVector = typename Base::UVector;

        /**
         * @brief 设置扫频信号参数。
         */
        void SetChirpInfo(const UVector& start_time, const UVector& end_time,
                          const UVector& start_frequency, const UVector& end_frequency,
                          const UVector& bias, const UVector& amplitude){
            this->start_time_ = start_time;
            this->end_time_ = end_time;
            this->duration_.noalias() = this->end_time_ - this->start_time_;
            this->start_frequency_ = start_frequency;
            this->end_frequency_ = end_frequency;
            this->bias_ = bias;
            this->amplitude_ = amplitude;
        };

        /**
         * @brief 设置采样周期。
         */
        void SetSamplingTime(const DataType& Ts){
            this->Ts_ = Ts;
        }


        /**
         * @brief 获取每个通道的起始时间。
         */
        const UVector& GetStartTime() const{
            return this->start_time_;
        };

        /**
         * @brief 获取每个通道的结束时间。
         */
        const UVector& GetEndTime() const{
            return this->end_time_;
        };

        /**
         * @brief 获取每个通道的起始频率。
         */
        const UVector& GetStartFrequency() const{
            return this->start_frequency_;
        };

        /**
         * @brief 获取每个通道的结束频率。
         */
        const UVector& GetEndFrequency() const{
            return this->end_frequency_;
        };

        /**
         * @brief 获取发生器当前内部时间。
         */
        const DataType& GetCurrentTime() const{
            return this->current_time_;
        };

        /**
         * @brief 生成一步扫频信号输出。
         */
        UVector Step(const Eigen::Ref<const YVector>& y_ref,
                     const Eigen::Ref<const YVector>& y,
                     const Eigen::Ref<const UVector>& u_min,
                     const Eigen::Ref<const UVector>& u_max) override;
        
        /**
         * @brief 更新指定通道的相位。
         */
        void CalculateTheta(int channel_id);

        /**
         * @brief 重置内部时间和相位。
         */
        void Reset() override{
            this->current_time_ = 0;
            this->theta_.setZero();
        };

        /**
         * @brief 初始化发生器。
         */
        void Initialize() override{
            this->Reset();
        }

        /**
         * @brief 终止发生器并清空内部状态。
         */
        void Terminate() override{
            this->Reset();
        }

    private:
        
        UVector start_time_{UVector::Zero()};
        UVector end_time_{UVector::Zero()};
        UVector duration_{UVector::Zero()};


        UVector theta_{UVector::Zero()};
        UVector start_frequency_{UVector::Zero()};
        UVector end_frequency_{UVector::Zero()};
        UVector bias_{UVector::Zero()};
        UVector amplitude_{UVector::Ones()};
        

        DataType Ts_{static_cast<DataType>(0.1)};
        DataType current_time_{static_cast<DataType>(0)};

};


/**
 * @brief 按线性扫频规律更新某个通道的相位。
 */
template<typename DataType, int Ny, int Nu>
void ChirpSignalGenerator<DataType, Ny, Nu>::CalculateTheta(int channel_id){

    DataType fk = this->start_frequency_(channel_id) + 
                    (this->end_frequency_(channel_id) - this->start_frequency_(channel_id))/this->duration_(channel_id) * 
                    (this->current_time_ - this->start_time_(channel_id));
    this->theta_(channel_id) += 2 * Pi<DataType> * fk * this->Ts_;
    this->theta_(channel_id) = std::fmod(this->theta_(channel_id), 2 * Pi<DataType>);

}


/**
 * @brief 输出当前采样时刻的扫频信号。
 */
template<typename DataType, int Ny, int Nu>
typename ChirpSignalGenerator<DataType, Ny, Nu>::UVector
ChirpSignalGenerator<DataType, Ny, Nu>::Step(
    const Eigen::Ref<const YVector>& y_ref,
    const Eigen::Ref<const YVector>& y,
    const Eigen::Ref<const UVector>& u_min,
    const Eigen::Ref<const UVector>& u_max) {


    (void) y_ref;
    (void) y;

    // 1. 计算当前时间
    this->current_time_ += this->Ts_;
    UVector u_unsat = UVector::Zero();
    
    // 2. 寻找对应位置并计算输出量
    for (int i = 0; i< Nu; i++){
        if (this->current_time_ < this->start_time_(i)){
            u_unsat(i) = this->bias_(i);
        }else if(this->current_time_ >= this->end_time_(i)){
            u_unsat(i) = this->bias_(i);
        }else{
            this->CalculateTheta(i);
            u_unsat(i) = this->bias_(i) + this->amplitude_(i) * std::sin(this->theta_(i));
        }
    }

    // 3. 控制量限幅 (Saturation)
    // 使用 Eigen 的 cwiseMin/cwiseMax 进行元素级限幅
    UVector u_sat = u_unsat.cwiseMax(u_min).cwiseMin(u_max);

    // 7. 返回实际执行的控制量
    return u_sat;
};




#pragma once

/**
 * @brief 圆周率常量。
 *
 * Parameters
 * ----------
 * DataType : typename
 *     标量类型，例如 float 或 double。
 *
 * Notes
 * -----
 * - 使用变量模板，方便在 float / double 之间切换。
 * - STM32F103 上建议使用 float。
 */
template<typename DataType>
constexpr DataType Pi = static_cast<DataType>(
    3.141592653589793238462643383279502884L
);
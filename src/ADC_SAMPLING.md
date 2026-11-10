# ADC 与光电传感器采样

三路 TCRT5000 光电传感器通过模拟输出识别编码条，黑色对应较高读数，白色对应较低读数。位于 `encoder.cpp`。

- **ADC 配置**：初始化时显式设置 12 位分辨率，并为每路设置 `ADC_0db` 衰减（如果黑白信号显著可提高衰减），使较低电压的传感器信号使用更合适的量程。
```cpp
analogSetPinAttenuation(aP, ADC_0db);
```
- **采样方式**：连续读取 8 次后求平均，四舍五入为整数，减小单次读数波动的影响。黑白判断与上传的模拟值均采用这种平均方式；每次调用都会重新采样。
```cpp
int TCRT5000::getaR() const {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_AVERAGE_SAMPLES; ++i) {
        sum += analogRead(aP);
    }
    return (sum + ADC_AVERAGE_SAMPLES / 2) / ADC_AVERAGE_SAMPLES;
}
```
- **黑白阈值**：平均读数大于 700 判黑，小于 650 判白，650～700（含边界）保持上一状态。阈值单位为 ADC 刻度。
```cpp
namespace {
    // Trial thresholds for 12-bit, 0 dB readings averaged over 8 samples.
    constexpr uint8_t ADC_AVERAGE_SAMPLES = 8;
    const int BLACK_THRESHOLD_A = 700;
    const int WHITE_THRESHOLD_A = 650;

    const int BLACK_THRESHOLD_B = 700;
    const int WHITE_THRESHOLD_B = 650;

    const int BLACK_THRESHOLD_C = 700;
    const int WHITE_THRESHOLD_C = 650;
}
```
阈值与当前 ADC 配置、安装距离及编码条有关，更换配置或安装条件后应重新确认黑白读数。

# ADC and Photoelectric Sensor Sampling

The three TCRT5000 photoelectric sensors identify the encoder strip through their analog output: black corresponds to a higher reading, white to a lower reading. Located in `encoder.cpp`.

- **ADC configuration**: 12-bit resolution is explicitly set at initialization, and `ADC_0db` attenuation is set for each channel (attenuation may be raised if the black/white signals are pronounced), so that the lower-voltage sensor signal is measured on a more appropriate range.
```cpp
analogSetPinAttenuation(aP, ADC_0db);
```
- **Sampling method**: Each reading is taken as the average of 8 consecutive samples, rounded to the nearest integer, to reduce the effect of single-sample fluctuation. Both the black/white decision and the uploaded analog value use this averaging; every call re-samples.
```cpp
int TCRT5000::getaR() const {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < ADC_AVERAGE_SAMPLES; ++i) {
        sum += analogRead(aP);
    }
    return (sum + ADC_AVERAGE_SAMPLES / 2) / ADC_AVERAGE_SAMPLES;
}
```
- **Black/white thresholds**: An averaged reading greater than 700 is judged black, less than 650 white, and 650–700 (boundaries included) holds the previous state. Thresholds are in ADC counts.
```cpp
namespace {
    // Trial thresholds for 12-bit, 0 dB readings averaged over 8 samples.
    constexpr uint8_t ADC_AVERAGE_SAMPLES = 8;
    const int BLACK_THRESHOLD_A = 700;
    const int WHITE_THRESHOLD_A = 650;

    const int BLACK_THRESHOLD_B = 700;
    const int WHITE_THRESHOLD_B = 650;

    const int BLACK_THRESHOLD_C = 700;
    const int WHITE_THRESHOLD_C = 650;
}
```
The thresholds depend on the current ADC configuration, the mounting distance and the encoder strip; the black/white readings should be re-confirmed after changing the configuration or the mounting conditions.

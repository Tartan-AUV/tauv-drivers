#include "tauv_depth/ms5837.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/i2c-dev.h>
#endif

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>

namespace tauv_depth
{

MS5837::MS5837(Model model, int bus)
: model_(model), bus_(bus)
{
  std::string device = "/dev/i2c-" + std::to_string(bus_);
  fd_ = ::open(device.c_str(), O_RDWR);
  if (fd_ >= 0) {
#ifdef __linux__
    if (::ioctl(fd_, I2C_SLAVE, MS5837_ADDR) < 0) {
      ::close(fd_);
      fd_ = -1;
    }
#endif
  }
}

MS5837::~MS5837()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool MS5837::writeCommand(uint8_t cmd)
{
  if (fd_ < 0) {
    return false;
  }
  return ::write(fd_, &cmd, 1) == 1;
}

bool MS5837::readBytes(uint8_t reg, uint8_t * data, size_t length)
{
  if (!writeCommand(reg)) {
    return false;
  }
  return ::read(fd_, data, length) == static_cast<ssize_t>(length);
}

bool MS5837::init()
{
  if (fd_ < 0) {
    return false;
  }

  if (!writeCommand(MS5837_RESET)) {
    return false;
  }

  // Wait 10 ms for reset to complete
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Read 7 PROM calibration words (big-endian on wire)
  for (size_t i = 0; i < 7; ++i) {
    uint8_t buf[2] = {0, 0};
    if (!readBytes(static_cast<uint8_t>(MS5837_PROM_READ + 2 * i), buf, 2)) {
      return false;
    }
    cal_[i] = static_cast<uint16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
  }
  cal_[7] = 0;

  uint8_t crc = static_cast<uint8_t>((cal_[0] & 0xF000) >> 12);
  if (crc != crc4(cal_)) {
    return false;
  }

  if (model_ == Model::MODEL_UNKNOWN) {
    autoDetectModel();
  }

  return model_ != Model::MODEL_UNKNOWN;
}

void MS5837::autoDetectModel()
{
  uint16_t pressure_sensitivity = cal_[1];
  if (pressure_sensitivity >= MS5837_30BA_MIN_SENSITIVITY &&
    pressure_sensitivity <= MS5837_02BA_30BA_SEPARATION)
  {
    model_ = Model::MODEL_30BA;
  } else if (pressure_sensitivity > MS5837_02BA_30BA_SEPARATION &&
    pressure_sensitivity <= MS5837_02BA_MAX_SENSITIVITY)
  {
    model_ = Model::MODEL_02BA;
  } else {
    model_ = Model::MODEL_UNKNOWN;
  }
}

bool MS5837::read(Oversampling oversampling)
{
  if (fd_ < 0) {
    return false;
  }

  uint8_t osr_idx = static_cast<uint8_t>(oversampling);
  if (osr_idx > static_cast<uint8_t>(Oversampling::OSR_8192)) {
    return false;
  }

  // Conversion delay: 2.5 us * 2^(8 + osr_idx)
  const auto conversion_delay = std::chrono::microseconds(
    static_cast<int64_t>(std::ceil(2.5 * static_cast<double>(1U << (8U + osr_idx)))));

  // Request D1 conversion (pressure)
  if (!writeCommand(static_cast<uint8_t>(MS5837_CONVERT_D1_256 + 2 * osr_idx))) {
    return false;
  }
  std::this_thread::sleep_for(conversion_delay);

  uint8_t d1_buf[3] = {0, 0, 0};
  if (!readBytes(MS5837_ADC_READ, d1_buf, 3)) {
    return false;
  }
  d1_ = (static_cast<uint32_t>(d1_buf[0]) << 16) |
    (static_cast<uint32_t>(d1_buf[1]) << 8) |
    static_cast<uint32_t>(d1_buf[2]);

  // Request D2 conversion (temperature)
  if (!writeCommand(static_cast<uint8_t>(MS5837_CONVERT_D2_256 + 2 * osr_idx))) {
    return false;
  }
  std::this_thread::sleep_for(conversion_delay);

  uint8_t d2_buf[3] = {0, 0, 0};
  if (!readBytes(MS5837_ADC_READ, d2_buf, 3)) {
    return false;
  }
  d2_ = (static_cast<uint32_t>(d2_buf[0]) << 16) |
    (static_cast<uint32_t>(d2_buf[1]) << 8) |
    static_cast<uint32_t>(d2_buf[2]);

  calculate();
  return true;
}

void MS5837::setFluidDensity(double density)
{
  fluid_density_ = density;
}

double MS5837::pressure(double conversion) const
{
  return pressure_ * conversion;
}

double MS5837::temperature(TemperatureUnit unit) const
{
  double deg_c = temperature_ / 100.0;
  if (unit == TemperatureUnit::FAHRENHEIT) {
    return (9.0 / 5.0) * deg_c + 32.0;
  } else if (unit == TemperatureUnit::KELVIN) {
    return deg_c + 273.0;
  }
  return deg_c;
}

double MS5837::depth() const
{
  return (pressure(UNITS_Pa) - 101300.0) / (fluid_density_ * 9.80665);
}

double MS5837::altitude() const
{
  return (1.0 - std::pow(pressure() / 1013.25, 0.190284)) * 145366.45 * 0.3048;
}

void MS5837::calculate()
{
  double off_i = 0.0;
  double sens_i = 0.0;
  double t_i = 0.0;

  double d_t = static_cast<double>(d2_) - static_cast<double>(cal_[5]) * 256.0;
  double sens = 0.0;
  double off = 0.0;

  if (model_ == Model::MODEL_02BA) {
    sens = static_cast<double>(cal_[1]) * 65536.0 + (static_cast<double>(cal_[3]) * d_t) / 128.0;
    off = static_cast<double>(cal_[2]) * 131072.0 + (static_cast<double>(cal_[4]) * d_t) / 64.0;
    pressure_ = (static_cast<double>(d1_) * sens / 2097152.0 - off) / 32768.0;
  } else if (model_ == Model::MODEL_30BA) {
    sens = static_cast<double>(cal_[1]) * 32768.0 + (static_cast<double>(cal_[3]) * d_t) / 256.0;
    off = static_cast<double>(cal_[2]) * 65536.0 + (static_cast<double>(cal_[4]) * d_t) / 128.0;
    pressure_ = (static_cast<double>(d1_) * sens / 2097152.0 - off) / 8192.0;
  } else {
    throw std::runtime_error("Cannot calculate pressure for unknown model type!");
  }

  temperature_ = 2000.0 + d_t * static_cast<double>(cal_[6]) / 8388608.0;

  // Second order compensation
  if (model_ == Model::MODEL_02BA) {
    if ((temperature_ / 100.0) < 20.0) {
      t_i = (11.0 * d_t * d_t) / 34359738368.0;
      double dt_temp = temperature_ - 2000.0;
      off_i = (31.0 * dt_temp * dt_temp) / 8.0;
      sens_i = (63.0 * dt_temp * dt_temp) / 32.0;
    }
  } else {
    if ((temperature_ / 100.0) < 20.0) {
      t_i = (3.0 * d_t * d_t) / 8589934592.0;
      double dt_temp = temperature_ - 2000.0;
      off_i = (3.0 * dt_temp * dt_temp) / 2.0;
      sens_i = (5.0 * dt_temp * dt_temp) / 8.0;
      if ((temperature_ / 100.0) < -15.0) {
        double dt_low = temperature_ + 1500.0;
        off_i = off_i + 7.0 * dt_low * dt_low;
        sens_i = sens_i + 4.0 * dt_low * dt_low;
      }
    } else {
      t_i = 2.0 * (d_t * d_t) / 137438953472.0;
      double dt_temp = temperature_ - 2000.0;
      off_i = (1.0 * dt_temp * dt_temp) / 16.0;
      sens_i = 0.0;
    }
  }

  double off2 = off - off_i;
  double sens2 = sens - sens_i;

  temperature_ -= t_i;
  if (model_ == Model::MODEL_02BA) {
    pressure_ = (((static_cast<double>(d1_) * sens2) / 2097152.0 - off2) / 32768.0) / 100.0;
  } else {
    pressure_ = (((static_cast<double>(d1_) * sens2) / 2097152.0 - off2) / 8192.0) / 10.0;
  }
}

uint8_t MS5837::crc4(std::array<uint16_t, 8> n_prom)
{
  uint16_t n_rem = 0;
  n_prom[0] &= 0x0FFF;
  n_prom[7] = 0;

  for (int i = 0; i < 16; ++i) {
    if (i % 2 == 1) {
      n_rem ^= static_cast<uint16_t>(n_prom[i >> 1] & 0x00FF);
    } else {
      n_rem ^= static_cast<uint16_t>(n_prom[i >> 1] >> 8);
    }

    for (int n_bit = 8; n_bit > 0; --n_bit) {
      if (n_rem & 0x8000) {
        n_rem = static_cast<uint16_t>((n_rem << 1) ^ 0x3000);
      } else {
        n_rem = static_cast<uint16_t>(n_rem << 1);
      }
    }
  }

  n_rem = static_cast<uint16_t>((n_rem >> 12) & 0x000F);
  return static_cast<uint8_t>(n_rem ^ 0x00);
}

}  // namespace tauv_depth


#ifndef TAUV_DEPTH__MS5837_HPP_
#define TAUV_DEPTH__MS5837_HPP_

#include <array>
#include <cstdint>

namespace tauv_depth
{

enum class Model : uint8_t
{
  MODEL_02BA = 0,
  MODEL_30BA = 1,
  MODEL_UNKNOWN = 255
};

enum class Oversampling : uint8_t
{
  OSR_256 = 0,
  OSR_512 = 1,
  OSR_1024 = 2,
  OSR_2048 = 3,
  OSR_4096 = 4,
  OSR_8192 = 5
};

enum class TemperatureUnit : uint8_t
{
  CENTIGRADE = 1,
  FAHRENHEIT = 2,
  KELVIN = 3
};

// Sensitivity thresholds for auto-detection
// Context: https://github.com/ArduPilot/ardupilot/pull/29122#issuecomment-2877269114
constexpr uint16_t MS5837_02BA_MAX_SENSITIVITY = 49000;
constexpr uint16_t MS5837_02BA_30BA_SEPARATION = 37000;
constexpr uint16_t MS5837_30BA_MIN_SENSITIVITY = 26000;

// Fluid densities (kg/m^3)
constexpr double DENSITY_FRESHWATER = 997.0;
constexpr double DENSITY_SALTWATER = 1029.0;

// Pressure conversion factors (from native unit, mbar)
constexpr double UNITS_Pa = 100.0;
constexpr double UNITS_hPa = 1.0;
constexpr double UNITS_kPa = 0.1;
constexpr double UNITS_mbar = 1.0;
constexpr double UNITS_bar = 0.001;
constexpr double UNITS_atm = 0.000986923;
constexpr double UNITS_Torr = 0.750062;
constexpr double UNITS_psi = 0.014503773773022;

class MS5837
{
public:
  explicit MS5837(Model model = Model::MODEL_UNKNOWN, int bus = 1);
  virtual ~MS5837();

  MS5837(const MS5837 &) = delete;
  MS5837 & operator=(const MS5837 &) = delete;

  bool init();
  bool read(Oversampling oversampling = Oversampling::OSR_8192);

  void setFluidDensity(double density);

  // Pressure in requested units (mbar * conversion)
  double pressure(double conversion = UNITS_mbar) const;

  // Temperature in requested units (default degrees C)
  double temperature(TemperatureUnit unit = TemperatureUnit::CENTIGRADE) const;

  // Depth relative to MSL pressure (101300 Pa) in given fluid density
  double depth() const;

  // Altitude relative to MSL pressure
  double altitude() const;

private:
  static constexpr uint8_t MS5837_ADDR = 0x76;
  static constexpr uint8_t MS5837_RESET = 0x1E;
  static constexpr uint8_t MS5837_ADC_READ = 0x00;
  static constexpr uint8_t MS5837_PROM_READ = 0xA0;
  static constexpr uint8_t MS5837_CONVERT_D1_256 = 0x40;
  static constexpr uint8_t MS5837_CONVERT_D2_256 = 0x50;

  void autoDetectModel();
  void calculate();
  static uint8_t crc4(std::array<uint16_t, 8> n_prom);

  bool writeCommand(uint8_t cmd);
  bool readBytes(uint8_t reg, uint8_t * data, size_t length);

  Model model_;
  int bus_;
  int fd_{-1};

  double fluid_density_{DENSITY_FRESHWATER};
  double pressure_{0.0};
  double temperature_{0.0};
  uint32_t d1_{0};
  uint32_t d2_{0};
  std::array<uint16_t, 8> cal_{};
};

class MS5837_30BA : public MS5837
{
public:
  explicit MS5837_30BA(int bus = 1)
  : MS5837(Model::MODEL_30BA, bus) {}
};

class MS5837_02BA : public MS5837
{
public:
  explicit MS5837_02BA(int bus = 1)
  : MS5837(Model::MODEL_02BA, bus) {}
};

}  // namespace tauv_depth

#endif  // TAUV_DEPTH__MS5837_HPP_


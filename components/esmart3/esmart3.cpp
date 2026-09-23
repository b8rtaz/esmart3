#include "esmart3.h"
#include "esphome/core/log.h"

namespace esphome {
namespace esmart3 {

static const char *TAG = "esmart3";

void ESmart3Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Esmart3:");

  LOG_SENSOR("  ", "Charge Mode", charge_mode_sensor_);
  LOG_SENSOR("  ", "Input Voltage", input_voltage_sensor_);
  LOG_SENSOR("  ", "Battery Voltage", battery_voltage_sensor_);
  LOG_SENSOR("  ", "Charging Current", charging_current_sensor_);
  LOG_SENSOR("  ", "Load Voltage", load_voltage_sensor_);
  LOG_SENSOR("  ", "Load Current", load_current_sensor_);
  LOG_SENSOR("  ", "Charging Power", charging_power_sensor_);
  LOG_SENSOR("  ", "Load Power", load_power_sensor_);
  LOG_SENSOR("  ", "Battery Temp", battery_temp_sensor_);
  LOG_SENSOR("  ", "Internal Temp", internal_temp_sensor_);
  LOG_SENSOR("  ", "Battery Level", battery_level_sensor_);

  LOG_SENSOR("  ", "Today Energy", today_energy_sensor_);
  LOG_SENSOR("  ", "Month Energy", month_energy_sensor_);
  LOG_SENSOR("  ", "Total Energy", total_energy_sensor_);
  LOG_SENSOR("  ", "Load Today Energy", load_today_energy_sensor_);
  LOG_SENSOR("  ", "Load Month Energy", load_month_energy_sensor_);
  LOG_SENSOR("  ", "Load Total Energy", load_total_energy_sensor_);

  this->check_uart_settings(9600);
}

void ESmart3Component::update() {
  if (receiving_) {
    ESP_LOGW(TAG, "Update interval overrun");
    return;
  }

  /*
   * db_ChgSts — dane bieżące:
   * AA 01 00 01 00 03 00 00 18 39
   */
  static uint8_t status_data[] = {
      0xAA, 0x01, 0x00, 0x01, 0x00,
      0x03, 0x00, 0x00, 0x18, 0x39
  };

  /*
   * db_Log od offsetu 0x0000, długość 0x1A:
   *
   * Zawiera:
   * - dwTodayEng  (offset 0x06)
   * - dwMonthEng  (offset 0x0A)
   *
   * AA 01 00 01 02 03 00 00 1A 35
   */
  static uint8_t log_first_data[] = {
      0xAA, 0x01, 0x00, 0x01, 0x02,
      0x03, 0x00, 0x00, 0x1A, 0x35
  };

  /*
   * db_Log od offsetu 0x000E, długość 0x10:
   *
   * Zawiera:
   * - dwTotalEng      offset 0x0E
   * - dwLoadTodayEng  offset 0x10
   * - dwLoadMonthEng  offset 0x12
   * - dwLoadTotalEng  offset 0x14
   *
   * AA 01 00 01 02 03 0E 00 10 31
   */
  static uint8_t log_second_data[] = {
      0xAA, 0x01, 0x00, 0x01, 0x02,
      0x03, 0x0E, 0x00, 0x10, 0x31
  };

  /*
   * Kolejność:
   * 0 = bieżące dane
   * 1 = produkcja dzienna / miesięczna
   * 2 = produkcja total / LOAD
   */
  static uint8_t request_type = 0;

  if (request_type == 0) {
    ESP_LOGD(TAG, "Requesting real-time status data");
    write_array(status_data, sizeof(status_data));
  } else if (request_type == 1) {
    ESP_LOGD(TAG, "Requesting daily and monthly PV energy");
    write_array(log_first_data, sizeof(log_first_data));
  } else {
    ESP_LOGD(TAG, "Requesting total PV and LOAD energy");
    write_array(log_second_data, sizeof(log_second_data));
  }

  request_type++;
  if (request_type > 2) {
    request_type = 0;
  }
}

void ESmart3Component::loop() {
  const uint32_t now = millis();

  if (receiving_ && (now - last_transmission_ >= 500)) {
    ESP_LOGW(TAG, "Last transmission too long ago. Reset RX index.");
    data_.clear();
    receiving_ = false;
  }

  if (!available())
    return;

  last_transmission_ = now;

  while (available()) {
    uint8_t c;
    read_byte(&c);

    if (!receiving_) {
      if (c != 0xAA)
        continue;

      receiving_ = true;
      data_.clear();
    }

    data_.push_back(c);

    // Bajt nr 6 określa długość payloadu odpowiedzi.
    if (data_.size() == 6)
      data_count_ = c;

    // Pełna ramka = 6 bajtów nagłówka + payload + checksum.
    if ((data_.size() > 6) && (data_.size() == data_count_ + 7)) {
      if (check_data_()) {
        parse_data_();
      }

      data_.clear();
      receiving_ = false;
    }
  }
}

bool ESmart3Component::check_data_() const {
  if (data_.size() < 7) {
    ESP_LOGW(TAG, "Response too short");
    return false;
  }

  // Odpowiedź regulatora ma CMD_SET_NO_RESP = 0x03.
  if (data_[3] != 0x03) {
    ESP_LOGW(TAG, "Unexpected response code: %d", data_[3]);
    return false;
  }

  uint8_t sum = 0;

  for (uint8_t c : data_) {
    sum += c;
  }

  if (sum != 0) {
    ESP_LOGW(TAG, "Data checksum failed");
    return false;
  }

  return true;
}

void ESmart3Component::parse_data_() {
  // data_[4] = Data Item ID.
  switch (data_[4]) {
    case 0x00:
      parse_status_data_();
      break;

    case 0x02:
      parse_log_data_();
      break;

    default:
      ESP_LOGW(TAG, "Unknown data item received: 0x%02X", data_[4]);
      break;
  }
}

void ESmart3Component::parse_status_data_() {
  if (data_.size() < 33) {
    ESP_LOGW(TAG, "Status response too short: %u bytes", data_.size());
    return;
  }

  const uint16_t charge_mode = get_16_bit_uint_(8);
  const float input_voltage = float(get_16_bit_uint_(10)) / 10.0f;
  const float battery_voltage = float(get_16_bit_uint_(12)) / 10.0f;
  const float charging_current = float(get_16_bit_uint_(14)) / 10.0f;

  // Indeks 16 to wOutVolt — parametr wewnętrzny, pomijamy.
  const float load_voltage = float(get_16_bit_uint_(18)) / 10.0f;
  const float load_current = float(get_16_bit_uint_(20)) / 10.0f;

  const uint16_t charging_power = get_16_bit_uint_(22);
  const uint16_t load_power = get_16_bit_uint_(24);
  const uint16_t battery_temp = get_16_bit_uint_(26);
  const uint16_t internal_temp = get_16_bit_uint_(28);
  const uint16_t battery_level = get_16_bit_uint_(30);

  ESP_LOGD(
      TAG,
      "Status: ChgMode=%d, PvVolt=%.1fV, BatVolt=%.1fV, ChgCurr=%.1fA, LoadVolt=%.1fV, LoadCurr=%.1fA",
      charge_mode,
      input_voltage,
      battery_voltage,
      charging_current,
      load_voltage,
      load_current);

  ESP_LOGD(
      TAG,
      "Status: ChgPower=%dW, LoadPower=%dW, BatTemp=%dC, InnerTemp=%dC, BatCap=%d%%",
      charging_power,
      load_power,
      battery_temp,
      internal_temp,
      battery_level);

  if (charge_mode_sensor_ != nullptr)
    charge_mode_sensor_->publish_state(charge_mode);

  if (input_voltage_sensor_ != nullptr)
    input_voltage_sensor_->publish_state(input_voltage);

  if (battery_voltage_sensor_ != nullptr)
    battery_voltage_sensor_->publish_state(battery_voltage);

  if (charging_current_sensor_ != nullptr)
    charging_current_sensor_->publish_state(charging_current);

  if (load_voltage_sensor_ != nullptr)
    load_voltage_sensor_->publish_state(load_voltage);

  if (load_current_sensor_ != nullptr)
    load_current_sensor_->publish_state(load_current);

  if (charging_power_sensor_ != nullptr)
    charging_power_sensor_->publish_state(charging_power);

  if (load_power_sensor_ != nullptr)
    load_power_sensor_->publish_state(load_power);

  if (battery_temp_sensor_ != nullptr)
    battery_temp_sensor_->publish_state(battery_temp);

  if (internal_temp_sensor_ != nullptr)
    internal_temp_sensor_->publish_state(internal_temp);

  if (battery_level_sensor_ != nullptr)
    battery_level_sensor_->publish_state(battery_level);
}

void ESmart3Component::parse_log_data_() {
  /*
   * W odpowiedzi db_Log:
   *
   * data_[6], data_[7] = offset odczytu.
   *
   * Jeśli offset = 0x0000:
   *   index 22 = dwTodayEng
   *   index 30 = dwMonthEng
   *
   * Jeśli offset = 0x000E:
   *   index 8  = dwTotalEng
   *   index 12 = dwLoadTodayEng
   *   index 16 = dwLoadMonthEng
   *   index 20 = dwLoadTotalEng
   *
   * Wszystkie wartości są Uint32 little-endian w Wh.
   * Dzielimy przez 1000, aby przekazać kWh do HA.
   */

  if (data_.size() < 15) {
    ESP_LOGW(TAG, "Energy log response too short: %u bytes", data_.size());
    return;
  }

  const uint16_t offset = get_16_bit_uint_(6);

  if (offset == 0x0000) {
    if (data_.size() < 35) {
      ESP_LOGW(TAG, "First energy log response too short: %u bytes", data_.size());
      return;
    }

    const float today_energy = float(get_32_bit_uint_(22)) / 1000.0f;
    const float month_energy = float(get_32_bit_uint_(30)) / 1000.0f;

    ESP_LOGD(
        TAG,
        "PV energy: Today=%.3f kWh, Month=%.3f kWh",
        today_energy,
        month_energy);

    if (today_energy_sensor_ != nullptr)
      today_energy_sensor_->publish_state(today_energy);

    if (month_energy_sensor_ != nullptr)
      month_energy_sensor_->publish_state(month_energy);

    return;
  }

  if (offset == 0x000E) {
    if (data_.size() < 25) {
      ESP_LOGW(TAG, "Second energy log response too short: %u bytes", data_.size());
      return;
    }

    const float total_energy = float(get_32_bit_uint_(8)) / 1000.0f;
    const float load_today_energy = float(get_32_bit_uint_(12)) / 1000.0f;
    const float load_month_energy = float(get_32_bit_uint_(16)) / 1000.0f;
    const float load_total_energy = float(get_32_bit_uint_(20)) / 1000.0f;

    ESP_LOGD(
        TAG,
        "Total energy: PVTotal=%.3f kWh, LoadToday=%.3f kWh, LoadMonth=%.3f kWh, LoadTotal=%.3f kWh",
        total_energy,
        load_today_energy,
        load_month_energy,
        load_total_energy);

    if (total_energy_sensor_ != nullptr)
      total_energy_sensor_->publish_state(total_energy);

    if (load_today_energy_sensor_ != nullptr)
      load_today_energy_sensor_->publish_state(load_today_energy);

    if (load_month_energy_sensor_ != nullptr)
      load_month_energy_sensor_->publish_state(load_month_energy);

    if (load_total_energy_sensor_ != nullptr)
      load_total_energy_sensor_->publish_state(load_total_energy);

    return;
  }

  ESP_LOGW(TAG, "Unknown db_Log offset received: 0x%04X", offset);
}

uint16_t ESmart3Component::get_16_bit_uint_(uint8_t start_index) const {
  return (uint16_t(this->data_[start_index + 1]) << 8) |
         uint16_t(this->data_[start_index]);
}

uint32_t ESmart3Component::get_32_bit_uint_(uint8_t start_index) const {
  return uint32_t(this->data_[start_index]) |
         (uint32_t(this->data_[start_index + 1]) << 8) |
         (uint32_t(this->data_[start_index + 2]) << 16) |
         (uint32_t(this->data_[start_index + 3]) << 24);
}

}  // namespace esmart3
}  // namespace esphome

/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <perching_arm/perching_arm.h>

namespace perching_arm {

// By default the arm is uninitialized
PerchingArm::PerchingArm()
    : serial_(std::bind(&PerchingArm::Read, this, std::placeholders::_1,
                        std::placeholders::_2)),
      cb_raw_data_({}),
      cb_sleep_ms_({}),
      rx_state_(PROTOCOL_WF_HEADER_1),
      rx_ptr_(0),
      rx_datalen_(0),
      telemetry_rx_(false),
      command_index_(0) {
  serial_.SetTimeoutCallback(std::bind(&PerchingArm::Timeout, this), 1000);
  serial_.SetShutdownCallback(std::bind(&PerchingArm::Shutdown, this));

  // Initialize params for gripper
  raw_.grip.adhesive_engage         = 0x00;
  raw_.grip.wrist_lock              = 0x00;
  raw_.grip.automatic_mode_enable   = 0x00;
  raw_.grip.experiment_in_progress  = 0x00;
  raw_.grip.file_is_open            = 0x00;
  raw_.grip.exp_idx                 = 65521;
  raw_.grip.delay_ms                = 200;    // hard coded on gripper
  raw_.grip.read_SD = false;
  for (size_t ii = 0; ii < 35; ii++) raw_.grip.line[ii] = '-';
}

// Connect to the arm
PerchingArmResult PerchingArm::Connect(std::string const &port, uint32_t baud,
                                       PerchingArmSleepMsCallback cb_sleep_ms,
                                       PerchingArmRawDataCallback cb_raw_data) {
  // Check the serial opens and return failure if not
  if (!serial_.Open(port, baud)) return RESULT_PORT_NOT_OPEN;
  // Make sure the DTR pin is set low be default
  serial_.SetResetPinState(ff_serial::RESET_PIN_LOW);
  // Save the callbacks
  if (cb_raw_data) {
    std::cout << "Connecting raw data callback" << std::endl;
    cb_raw_data_ = cb_raw_data;
  }
  if (cb_sleep_ms) cb_sleep_ms_ = cb_sleep_ms;
  // Success!
  return RESULT_SUCCESS;
}

// Disconnect from the arm
void PerchingArm::Disconnect() {
  serial_.Close();
  cb_raw_data_ = nullptr;
  cb_sleep_ms_ = nullptr;
}

bool PerchingArm::ResultToString(PerchingArmResult result, std::string &msg) {
  bool success = (result == RESULT_SUCCESS);
  switch (result) {
    case RESULT_SUCCESS:
      msg = "Success";
      break;
    case RESULT_PORT_NOT_OPEN:
      msg = "Port not open";
      break;
    case RESULT_COMMAND_REJECTED:
      msg = "Command rejected";
      break;
    case RESULT_INVALID_COMMAND:
      msg = "Invalid command";
      break;
    case RESULT_PORT_WRITE_FAILURE:
      msg = "Port write failure";
      break;
    case RESULT_PORT_INIT_FAILURE:
      msg = "Port initialize failure";
      break;
    case RESULT_FIRMWARE_ERROR:
      msg = "Firmware error";
      break;
    case RESULT_RESPONSE_TIMEOUT:
      msg = "Response timeout";
      break;
    case RESULT_OUT_OF_BOUNDS:
      msg = "Out of bounds";
      break;
  }
  return success;
}

// Set the velocity of the proximal joint in revolutions per minute
PerchingArmResult PerchingArm::SetProximalVelocity(int16_t rpm) {
  std::cout << "Setting proximal joint to a certain velocity" << std::endl;
  if (rpm < PROX_VEL_MIN || rpm > PROX_VEL_MAX) return RESULT_OUT_OF_BOUNDS;
  // convert units from rpm to 0.1rpm
  rpm = 10.0 * rpm;
  return SendCommand(HOST_ARM_BASIC_CMD_TILT_JOINT, ADDRESS_VELOCITY, rpm);
}

// Set the position of the proximal joint in whole degrees
PerchingArmResult PerchingArm::SetProximalPosition(int16_t deg) {
  std::cout << "Setting proximal joint to a certain position" << std::endl;
  if (deg < PROX_POS_MIN || deg > PROX_POS_MAX) return RESULT_OUT_OF_BOUNDS;
  // convert units from deg to 0.1deg
  deg = deg * 10;
  return SendCommand(HOST_ARM_BASIC_CMD_TILT_JOINT, ADDRESS_POSITION, deg);
}

// Enable or disabled the proximal motor
PerchingArmResult PerchingArm::SetProximalEnabled() {
  std::cout << "Setting proximal joint to enabled" << std::endl;
  return SendCommand(HOST_ARM_BASIC_CMD_TILT_JOINT, ADDRESS_ENABLE, 0);
}

PerchingArmResult PerchingArm::SetProximalDisabled() {
  std::cout << "Setting proximal joint to disabled" << std::endl;
  return SendCommand(HOST_ARM_BASIC_CMD_TILT_JOINT, ADDRESS_DISABLE, 0);
}

// Set the velocity of the distal joint in revolutions per minute
PerchingArmResult PerchingArm::SetDistalVelocity(int16_t rpm) {
  std::cout << "Setting distal joint to a certain velocity" << std::endl;
  if (rpm < DIST_VEL_MIN || rpm > DIST_VEL_MAX) return RESULT_OUT_OF_BOUNDS;
  // convert units from rpm to 0.1rpm
  rpm = 10.0 * rpm;
  return SendCommand(HOST_ARM_BASIC_CMD_PAN_JOINT, ADDRESS_VELOCITY, rpm);
}

// Set the position of the distal joint in whole degrees
PerchingArmResult PerchingArm::SetDistalPosition(int16_t deg) {
  std::cout << "Setting distal joint to a certain position" << std::endl;
  if (deg < DIST_POS_MIN || deg > DIST_POS_MAX) return RESULT_OUT_OF_BOUNDS;
  // convert units from deg to 0.1deg
  deg = deg * 10;
  return SendCommand(HOST_ARM_BASIC_CMD_PAN_JOINT, ADDRESS_POSITION, deg);
}

// Enable or disabled the distal motor
PerchingArmResult PerchingArm::SetDistalEnabled() {
  std::cout << "Setting distal joint to enabled" << std::endl;
  return SendCommand(HOST_ARM_BASIC_CMD_PAN_JOINT, ADDRESS_ENABLE, 0);
}

PerchingArmResult PerchingArm::SetDistalDisabled() {
  std::cout << "Setting distal joint to disabled" << std::endl;
  return SendCommand(HOST_ARM_BASIC_CMD_PAN_JOINT, ADDRESS_DISABLE, 0);
}

PerchingArmResult PerchingArm::OpenGripper() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_OPEN, 0x00);
}

PerchingArmResult PerchingArm::CloseGripper() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_CLOSE, 0x00);
}

PerchingArmResult PerchingArm::EngageGripper() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_ENGAGE, 0x00);
}

PerchingArmResult PerchingArm::DisengageGripper() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_DISENGAGE, 0x00);
}

PerchingArmResult PerchingArm::LockGripper() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_LOCK, 0x00);
}

PerchingArmResult PerchingArm::UnlockGripper() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_UNLOCK, 0x00);
}

PerchingArmResult PerchingArm::EnableAutoGripper() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_ENABLE_AUTO, 0x00);
}

PerchingArmResult PerchingArm::DisableAutoGripper() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_DISABLE_AUTO, 0x00);
}

PerchingArmResult PerchingArm::ToggleAutoGripper() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_TOGGLE_AUTO, 0x00);
}

PerchingArmResult PerchingArm::MarkGripper(int16_t idx) {
  int16_t IDX;
  IDX = ((idx & 0xFF) << 8) | ((idx & 0xFF00) >> 8);
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_MARK, IDX);
}

PerchingArmResult PerchingArm::SetDelayGripper(int16_t delay_ms) {
  int16_t DL;
  DL = ((delay_ms & 0xFF) << 8) | ((delay_ms & 0xFF00) >> 8);
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_SET_DELAY, DL);
}

PerchingArmResult PerchingArm::OpenExperimentGripper(int16_t idx) {
  int16_t IDX;
  IDX = ((idx & 0xFF) << 8) | ((idx & 0xFF00) >> 8);
  raw_.grip.read_SD = true;
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_OPEN_EXPERIMENT, IDX);
}

PerchingArmResult PerchingArm::NextRecordGripper(int16_t skip) {
  int16_t SKIP;
  SKIP = ((skip & 0xFF) << 8) | ((skip & 0xFF00) >> 8);
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_NEXT_RECORD, SKIP);
}

PerchingArmResult PerchingArm::SeekRecordGripper(int16_t record_num) {
  int16_t RN;
  RN = ((record_num & 0xFF) << 8) | ((record_num & 0xFF00) >> 8);
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_SEEK_RECORD, RN);
}

PerchingArmResult PerchingArm::CloseExperimentGripper() {
  raw_.grip.read_SD = false;
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, ADDRESS_GRIPPER_CLOSE_EXPERIMENT, 0x00);
}

PerchingArmResult PerchingArm::Status() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, GECKO_REG_PRESENT_STATUS, 0x00);
}

PerchingArmResult PerchingArm::Record() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, GECKO_REG_PRESENT_RECORD, 0x00);
}

PerchingArmResult PerchingArm::ExperimentIdx() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, GECKO_REG_PRESENT_EXPERIMENT, 0x00);
}

PerchingArmResult PerchingArm::PresentDelay() {
  return SendCommand(HOST_ARM_BASIC_CMD_GECKO_GRIPPER, GECKO_REG_PRESENT_DELAY, 0x00);
}

// TODO(acauligi): check method with Tony
void PerchingArm::ConstructDataPacket(double* data, size_t data_len) {
  // Clear data held inside array
  for (size_t ii = 0; ii < data_len; ii++) {
    data[ii] = 0.0;
  }

  if (raw_.grip.read_SD) {
    data[0] = 1.;
    for (size_t jj = 0; jj < 35; jj++) {
      data[1+jj] = raw_.grip.line[jj];
      raw_.grip.line[jj] = '-';
    }
  } else {
    data[0] = 0.0;
    data[1] = raw_.grip.error_status;
    data[2] = raw_.grip.last_status_read_time;
    data[3] = raw_.grip.overtemperature_flag;
    data[4] = raw_.grip.experiment_in_progress;
    data[5] = raw_.grip.file_is_open;
    data[6] = raw_.grip.automatic_mode_enable;
    data[7] = raw_.grip.wrist_lock;
    data[8] = raw_.grip.adhesive_engage;
    data[9] = raw_.grip.delay_ms;
    data[10] = raw_.grip.exp_idx;
  }
}

// Reset the software
PerchingArmResult PerchingArm::SoftReset() {
  std::cout << "Sending a soft reset command" << std::endl;
  return SendCommand(HOST_ARM_BASIC_CMD_RESET, ADDRESS_RESET, 0);
}

// Reset the hardware
PerchingArmResult PerchingArm::HardReset() {
  std::cout << "Performing a hard reset" << std::endl;
  // Check serial is open
  if (!serial_.IsOpen()) return RESULT_PORT_NOT_OPEN;
  // Reset the board
  serial_.SetResetPinState(ff_serial::RESET_PIN_HIGH);
  cb_sleep_ms_(500);
  serial_.SetResetPinState(ff_serial::RESET_PIN_LOW);
  return RESULT_SUCCESS;
}

// Blocking write
PerchingArmResult PerchingArm::SendCommand(uint8_t target, int16_t address,
                                           int16_t data) {
  if (!serial_.IsOpen()) return RESULT_PORT_NOT_OPEN;

  command_index_++;

  host_arm_basic_cmd_t packet;
  uint8_t *packet_data = reinterpret_cast<uint8_t *>(&packet);

  packet.prefix.header.leadin = {HOST_ARM_CMD_LEADIN_1, HOST_ARM_CMD_LEADIN_2};
  packet.prefix.header.length =
      sizeof(packet) - sizeof(host_arm_cmd_header_t) + sizeof(uint8_t);

  packet.prefix.type = target;
  packet.prefix.index = command_index_;

  packet.address = htole16(address);
  packet.data = htole16(data);

  size_t size = sizeof(packet);
  uint8_t checksum = Checksum(packet_data, size);

  for (uint8_t nwritten = 0; nwritten < size;)
    nwritten += serial_.Write(packet_data + nwritten, size - nwritten);

  for (uint8_t nwritten = 0; nwritten < sizeof(checksum);)
    nwritten +=
        serial_.Write(&checksum + nwritten, sizeof(checksum) - nwritten);

  return RESULT_SUCCESS;
}

void PerchingArm::ProcessGeckoGripperStateTelemetry(
    const host_gecko_gripper_state_telemetry_t *packet) {

  // TODO(acauligi): Anyway to retain backwards compatibility here?
  raw_.grip.error_status            = packet->error_status;
  uint8_t err;
  if (raw_.grip.error_status && 0x80) {
    // Check if first bit is 1
    err = raw_.grip.error_status & 0x7F;
    switch (err) {
      case perching_arm::ERR_RESULT:
        std::cout << "Result Fail" << std::endl;
      case perching_arm::ERR_INSTR:
        std::cout << "Instruction Fail" << std::endl;
      case perching_arm::ERR_CRC:
        std::cout << "CRC Error" << std::endl;
      case perching_arm::ERR_DATA_RANGE:
        std::cout << "Data Range Error" << std::endl;
      case perching_arm::ERR_DATA_LEN:
        std::cout << "Data Length Error" << std::endl;
      case perching_arm::ERR_DATA_LIM:
      std::cout << "Data Limit Error" << std::endl;
      case perching_arm::ERR_ACCESS:
        std::cout << "Access Error" << std::endl;
      case perching_arm::ERR_INSTR_READ:
        std::cout << "Instruction Read Error" << std::endl;
      case perching_arm::ERR_INSTR_WRITE:
        std::cout << "Instruction Write Error" << std::endl;
      case perching_arm::ERR_TOF_INIT:
        std::cout << "TOF Init Error" << std::endl;
      case perching_arm::ERR_TOF_READ:
        std::cout << "TOF Read Error" << std::endl;
      case perching_arm::ERR_SD_INIT:
        std::cout << "SD Init Error" << std::endl;
      case perching_arm::ERR_SD_OPEN:
        std::cout << "SD Open Error" << std::endl;
      case perching_arm::ERR_SD_WRITE:
        std::cout << "SD Write Error" << std::endl;
      case perching_arm::ERR_SD_READ:
        std::cout << "SD Read Error" << std::endl;
      default:
        std::cout << "Unrecognized error flag!" << std::endl;
        break;
    }
  } else {
    // If first bit = 0, ignore trailing bits
    err = 0x00;
  }
  err = raw_.grip.error_status & 0x7F;

  raw_.grip.last_status_read_time   = packet->last_status_read_time;  // time since gripper was polled
  raw_.grip.adhesive_engage         = packet->adhesive_engage;  // bit for adhesive engage/disengage
  raw_.grip.wrist_lock              = packet->wrist_lock;  // bit for wrist lock/unlock
  raw_.grip.automatic_mode_enable   = packet->automatic_mode_enable;  // bit for automatic mode enable/disable
  raw_.grip.experiment_in_progress  = packet->experiment_in_progress;  // bit to indicate experiment in progress
  raw_.grip.overtemperature_flag    = packet->overtemperature_flag;  // bit to indicate overtemperature

  raw_.grip.file_is_open            = packet->file_is_open;
  raw_.grip.exp_idx                 = packet->exp_idx;
  raw_.grip.delay_ms                = packet->delay_ms;
}

void PerchingArm::ProcessSDCardStateTelemetry(
    const host_sd_card_state_telemetry_t *packet) {
  for (size_t ii = 0; ii < 35; ii++) {
    raw_.grip.line[ii] = packet->line[ii];
  }
  // for (size_t ii = 0; ii < 35; ii++) {
  //   std::cout << raw_.grip.line[ii];;
  // }
  // std::cout << std::endl;
}

void PerchingArm::ProcessServoJointStateTelemetry(
    const host_arm_servo_joint_state_telemetry_t *packet) {
  // NOTE: prox = tilt, dist = pan
  raw_.prox.load = packet->tilt_state.current;       // unit: 0.1mA
  raw_.prox.velocity = packet->tilt_state.velocity;  // unit: 0.1RPM
  raw_.prox.position = packet->tilt_state.position;  // unit: 0.1deg
  raw_.dist.load = packet->pan_state.current;        // unit: 0.1mA
  raw_.dist.velocity = packet->pan_state.velocity;   // unit: 0.1RPM
  raw_.dist.position = packet->pan_state.position;   // unit: 0.1deg
}

void PerchingArm::ProcessServoJointStatusTelemetry(
    const host_arm_servo_joint_status_telemetry_t *packet) {
  // pass
}

void PerchingArm::ProcessBoardStateTelemetry(
    const host_arm_board_state_telemetry_t *packet) {
  raw_.current_11v = packet->avg_v11_current;  // unit: 4.297mA
  // warning! 5v/6v mapping must be confirmed later
  raw_.current_5v = packet->avg_v06_current;  // unit: 3.223mA
  raw_.board_temp = packet->temperature;      // unit: 0.1 degC
}

void PerchingArm::ProcessBoardInfoTelemetry(
    const host_arm_board_info_telemetry_t *packet) {
  // pass
}

void PerchingArm::ProcessErrorMsgTelemetry(
    const host_arm_error_msg_telemetry_t *packet) {
  std::cout << "Error! message:" << std::endl;
  std::cout << packet->msg << std::endl;
}

void PerchingArm::Process(const uint8_t *buffer) {
  // header (lead-in + length) has already been verified

  const host_arm_telemetry_prefix_t *prefix =
      reinterpret_cast<const host_arm_telemetry_prefix_t *>(buffer);

  if (prefix->header.leadin.leadin1 != 0xFF) {
    std::cout << "leadin1 check failed" << std::endl;
    return;
  }

  if (prefix->header.leadin.leadin2 != 0xFD) {
    std::cout << "leadin2 check failed" << std::endl;
    return;
  }

  switch (prefix->type) {
    case HOST_ARM_TELEMETRY_SERVO_JOINT_STATE:
      ProcessServoJointStateTelemetry(
          (const host_arm_servo_joint_state_telemetry_t *)buffer);
      break;
    case HOST_ARM_TELEMETRY_SERVO_JOINT_STATUS:
      ProcessServoJointStatusTelemetry(
          (const host_arm_servo_joint_status_telemetry_t *)buffer);
      break;
    case HOST_ARM_TELEMETRY_GECKO_GRIPPER_STATE :
      ProcessGeckoGripperStateTelemetry(
          (const host_gecko_gripper_state_telemetry_t*)buffer);
      break;
    case HOST_ARM_TELEMETRY_SD_CARD_STATE :
      ProcessSDCardStateTelemetry(
          (const host_sd_card_state_telemetry_t*)buffer);
      break;
    case HOST_ARM_TELEMETRY_BOARD_STATE:
      ProcessBoardStateTelemetry(
          (const host_arm_board_state_telemetry_t *)buffer);
      break;
    case HOST_ARM_TELEMETRY_BOARD_INFO:
      ProcessBoardInfoTelemetry(
          (const host_arm_board_info_telemetry_t *)buffer);
      break;
    case HOST_ARM_TELEMETRY_PONG:
      return;
    case HOST_ARM_TELEMETRY_ACK:
      return;
    case HOST_ARM_TELEMETRY_LOG_MSG:
      return;
    case HOST_ARM_TELEMETRY_ERROR_MSG:
      ProcessErrorMsgTelemetry((const host_arm_error_msg_telemetry_t *)buffer);
      break;

    default:
      std::cout << "Error: type matched nothing!" << std::endl;
      return;
  }

  // Mark that we have received data from the arm
  telemetry_rx_ = true;

  // Push the raw data
  if (cb_raw_data_) cb_raw_data_(raw_);
}

// Callback with serial data
void PerchingArm::Read(const uint8_t *buf, size_t len) {
  for (uint i = 0; i < len; i++) FiniteStateMachine(buf[i]);
}

// Called when we time out
void PerchingArm::Timeout(void) {
  std::cout << "Perching arm has timed out" << std::endl;
}

// Called when we time out
void PerchingArm::Shutdown(void) {
  std::cout << "Perching arm has shutdown" << std::endl;
}

// Process a single character as part of the protocol state machine
void PerchingArm::FiniteStateMachine(uint8_t const &c) {
  // If we are waiting for header 1, then the packet pointer is reset
  if (rx_state_ == PROTOCOL_WF_HEADER_1) rx_ptr_ = 0;
  // Copy the data
  if (rx_ptr_ < MAX_PACKET_SIZE) rx_buf_[rx_ptr_++] = c;
  // Advance the state machine
  switch (rx_state_) {
    // Wait for header 1
    case PROTOCOL_WF_HEADER_1:
      if (c != 0xFF) {
        rx_state_ = PROTOCOL_WF_HEADER_1;
        return;
      }
      rx_state_ = PROTOCOL_WF_HEADER_2;
      return;
    // Wait for header 2
    case PROTOCOL_WF_HEADER_2:
      if (c != 0xFD) {
        rx_state_ = PROTOCOL_WF_HEADER_1;
        return;
      }
      rx_state_ = PROTOCOL_WF_LENGTH;
      return;
    // Wait for packet length
    case PROTOCOL_WF_LENGTH:
      rx_datalen_ = c - 1;
      rx_state_ = PROTOCOL_WF_DATA;
      return;
    // Wait until all data rx
    case PROTOCOL_WF_DATA:
      if (--rx_datalen_) return;
      rx_state_ = PROTOCOL_WF_CHECKSUM;
      return;
    // Wait for checksum
    case PROTOCOL_WF_CHECKSUM:
      if (Checksum(rx_buf_, rx_ptr_ - 1) == rx_buf_[rx_ptr_ - 1])
        Process(&rx_buf_[0]);
      rx_state_ = PROTOCOL_WF_HEADER_1;
      return;
  }
}

// Checksum calculation
uint8_t PerchingArm::Checksum(const uint8_t *buffer, size_t size) {
  uint8_t sum = 0;  // CRC8 initialization value = 0
  for (size_t i = 0; i < size; i++)
    sum = host_arm_comm_crc8_table[sum ^ buffer[i]];
  return sum;
}

}  // namespace perching_arm

// librosco - a communications library for the Rover MEMS ECU
//
// protocol.c: This file contains routines specific to handling
//             the software protocol used by the ECU over its
//             serial link.

#if defined(WIN32) && defined(linux)
#error "Only one of 'WIN32' or 'linux' may be defined."
#endif

#include <unistd.h>
#include <stdio.h>
#include <string.h>

#if defined(WIN32)
  #include <windows.h>
#elif defined(__NetBSD__)
  #include <string.h>
#endif

#include "rosco.h"
#include "rosco_internal.h"

/**
 * Reads bytes from the serial device using an OS-specific call.
 * @param buffer Buffer into which data should be read
 * @param quantity Number of bytes to read
 * @return Number of bytes read from the device, or -1 if no bytes could be read
 */
int16_t mems_read_serial(mems_info* info, uint8_t* buffer, uint16_t quantity)
{
  int16_t totalBytesRead = 0;
  int16_t bytesRead = -1;
  uint8_t *buffer_pt = buffer;

  if (mems_is_connected(info))
  {
    do
    {
      DWORD ftBytesRead = 0;
      if ((FT_Read(info->ft, buffer_pt, quantity, &ftBytesRead) == FT_OK) &&
          (ftBytesRead > 0))
      {
        bytesRead = ftBytesRead;
      }

      totalBytesRead += bytesRead;
      buffer_pt += bytesRead;

    } while ((bytesRead > 0) && (totalBytesRead < quantity));
  }

  if (totalBytesRead > 0)
  {
    dprintf_err("mems_read_serial(): read %d bytes, expected %d:", totalBytesRead, quantity);
    for (uint8_t i = 0U; i < totalBytesRead; ++i)
    {
      dprintf_err(" %02X", buffer[i]);
    }
    dprintf_err("\n");
  }

  if (totalBytesRead < quantity)
  {
    dprintf_err("mems_read_serial(): expected %d, got %d\n", quantity, totalBytesRead);
  }

  return totalBytesRead;
}

/**
 * Writes bytes to the serial device using an OS-specific call
 * @param buffer Buffer from which written data should be drawn
 * @param quantity Number of bytes to write
 * @return Number of bytes written to the device, or -1 if no bytes could be written
 */
int16_t mems_write_serial(mems_info* info, uint8_t* buffer, uint16_t quantity)
{
  int16_t bytesWritten = -1;

  {
    dprintf_err("mems_write_serial(): writing %d bytes:", quantity);
    for (uint32_t i = 0U; i < quantity; ++i)
    {
      dprintf_err(" %02X", buffer[i]);
    }
    dprintf_err("\n");
  }

  if (mems_is_connected(info))
  {
    DWORD ftBytesWritten = 0;
    if ((FT_Write(info->ft, buffer, quantity, &ftBytesWritten) == FT_OK) &&
        (ftBytesWritten == quantity))
    {
      bytesWritten = ftBytesWritten;
    }
  }

  return bytesWritten;
}

/**
 * Sets a break on the serial TX line (takes it low) for a period of
 * microseconds before disabling again and waiting for the same period.
 */
void mems_break_serial(mems_info* info, uint32_t duration_us)
{
  uint8_t response = 0xFF;

  if (FT_SetBreakOn(info->ft) == FT_OK)
  {
    wait_for_us(duration_us);

    if (FT_SetBreakOff(info->ft) == FT_OK)
    {
      wait_for_us(duration_us);
    }
    else
    {
      dprintf_err("mems_break_serial(): Failed to turn break off\n");
    }
  }
  else
  {
    dprintf_err("mems_break_serial(): Failed to turn break on\n");
  }

  // The break looks like an 0x00 byte so need to read one back
  if (mems_read_serial(info, &response, 1) != 1)
  {
    dprintf_err("mems_break_serial(): Failed to read 0x00\n");
  }
}

/**
 * Sends a single command byte to the ECU and waits for the same byte to be
 * echoed as a response. Note that if the ECU sends one or more bytes of
 * data in addition to the echoed command byte, mems_read_serial() must also
 * be called to retrieve that data from the input buffer.
 */
bool mems_send_command(mems_info *info, uint8_t cmd)
{
  bool result = false;
  uint8_t response = 0xFF;

  if (mems_write_serial(info, &cmd, 1) == 1)
  {
    if (mems_read_serial(info, &response, 1) == 1)
    {
      if (response == cmd)
      {
        result = true;
      }
      else
      {
        dprintf_err("mems_send_command(): received one nonmatching byte (%02X) in response to command %02X\n", response, cmd);
      }
    }
    else
    {
      dprintf_err("mems_send_command(): did not receive echo of command %02X\n", cmd);
    }
  }
  else
  {
    dprintf_err("mems_send_command(): failed to send command %02X\n", cmd);
  }

  return result;
}

/**
 * Sends a multi-byte command to the ECU along with a checksum.
 */
bool mems_send_command_with_checksum(mems_info* info, uint8_t* cmd_bytes, uint8_t num_bytes)
{
  bool result = true;
  uint8_t checksum = 0U;
  uint8_t byte_idx = 0U;
  static const uint32_t byte_delay_us = 4200U;

  // Commands should be spaced at least 50ms apart
  static const uint32_t command_delay_us = 50000U;
  wait_until_us(info->last_command_us + command_delay_us);

  dprintf_err("mems_send_command_with_checksum(): sending command...\n");

  // Send the command
  uint32_t last_us = get_current_us();
  while (byte_idx < num_bytes && result)
  {
    if (mems_write_serial(info, &cmd_bytes[byte_idx], 1) == 1)
    {
      checksum += cmd_bytes[byte_idx];
      last_us = wait_until_us(last_us + byte_delay_us);
    }
    else
    {
      dprintf_err("mems_send_command_with_checksum(): failed to send byte %d (%02X) of command\n", byte_idx, cmd_bytes[byte_idx]);
      result = false;
    }

    ++byte_idx;
  }

  dprintf_err("mems_send_command_with_checksum(): sending checksum...\n");

  // Send the command checksum
  if (mems_write_serial(info, &checksum, 1) != 1)
  {
    dprintf_err("mems_send_command_with_checksum(): failed to send checksum byte %02X\n", checksum);
    result = false;
  }

  dprintf_err("mems_send_command_with_checksum(): reading back echoed bytes...\n");

  // Read back the echoed bytes
  uint8_t echoed_byte = 0x00;
  byte_idx = 0U;
  while (byte_idx < num_bytes && result)
  {
    if (mems_read_serial(info, &echoed_byte, 1) == 1)
    {
      if (echoed_byte != cmd_bytes[byte_idx])
      {
        dprintf_err("mems_send_command_with_checksum(): echoed byte (%02X) did not match byte %d (%02X)\n", echoed_byte, byte_idx, cmd_bytes[byte_idx]);
        result = false;
      }
    }
    else
    {
      dprintf_err("mems_send_command_with_checksum(): did not receive echo of byte %d (%02X)\n", byte_idx, cmd_bytes[byte_idx]);
      result = false;
    }

    ++byte_idx;
  }

  dprintf_err("mems_send_command_with_checksum(): reading back echoed checksum...\n");

  // Read back the checksum
  if (mems_read_serial(info, &echoed_byte, 1) == 1)
  {
    if (echoed_byte != checksum)
    {
      dprintf_err("mems_send_command_with_checksum(): echoed checksum (%02X) did not match sent checksum (%02X)\n", echoed_byte, checksum);
      result = false;
    }
  }
  else
  {
    dprintf_err("mems_send_command_with_checksum(): did not receive echo of checksum (%02X)\n", checksum);
    result = false;
  }

  return result;
}

/**
 * Receives a multi-byte response with a checksum from the ECU.
 */
bool mems_read_response_with_checksum(mems_info* info, uint8_t* response, uint8_t num_bytes)
{
  bool result = false;
  uint8_t checksum = 0;
  uint8_t byte_idx = 0;

  dprintf_err("mems_read_response_with_checksum(): reading length byte...\n");

  // Read the first byte which is the response length
  uint8_t length_byte = 0x00;
  if (mems_read_serial(info, &length_byte, 1) == 1)
  {
    if (length_byte == num_bytes)
    {
      dprintf_err("mems_read_response_with_checksum(): reading response bytes...\n");

      // Read the response
      if (mems_read_serial(info, response, num_bytes) == num_bytes)
      {
        dprintf_err("mems_read_response_with_checksum(): reading checksum byte...\n");

        // Read the checksum
        uint8_t checksum_byte = 0x00;
        if (mems_read_serial(info, &checksum_byte, 1) == 1)
        {
          // Calculate and validate the checksum
          uint8_t calculated_checksum = length_byte;
          for (byte_idx = 0; byte_idx < num_bytes; ++byte_idx)
          {
            calculated_checksum += response[byte_idx];
          }
          if (checksum_byte == calculated_checksum)
          {
            // The correct number of response bytes were returned and
            // the checksum was valid!
            result = true;
          }
          else
          {
            dprintf_err("mems_read_response_with_checksum(): invalid checksum %02X, expected %02X\n", checksum_byte, calculated_checksum);
          }
        }
        else
        {
          dprintf_err("mems_read_response_with_checksum(): failed to read checkum byte\n");
        }
      }
      else
      {
        dprintf_err("mems_read_response_with_checksum(): read incorrect number of bytes\n");
      }
    }
    else
    {
      dprintf_err("mems_read_response_with_checksum(): read incorrect length response: %02X", length_byte);
      // Make sure the response is still consumed (including checksum)
      uint8_t consumed_byte = 0x00;
      length_byte++;
      while (length_byte)
      {
        if (mems_read_serial(info, &consumed_byte, 1) == 1)
        {
          dprintf_err(" %02X", consumed_byte);
        }
        else
        {
          dprintf_err(" ??");
        }
      }
    }
  }
  else
  {
    dprintf_err("mems_read_response_with_checksum(): failed to read response length byte\n");
  }

  // Commands should be spaced apart from the last response byte received
  info->last_command_us = get_current_us();

  return result;
}

/**
 * Sends an initialization/startup sequence to the ECU. Required to enable further communication.
 */
bool mems_init_link(mems_info* info, uint8_t* response_buffer)
{
  switch (info->ver)
  {
    default:
    MEMS_Version_16:
    {
      uint8_t command_a = 0xCA;
      uint8_t command_b = 0x75;
      uint8_t command_c = MEMS_Heartbeat;
      uint8_t command_d = 0xD0;
      uint8_t buffer = 0x00;

      if (!mems_send_command(info, command_a))
      {
        dprintf_err("mems_init_link(): Did not see %02X command echo\n", command_a);
        return false;
      }
      if (!mems_send_command(info, command_b))
      {
        dprintf_err("mems_init_link(): Did not see %02X command echo\n", command_b);
        return false;
      }
      if (!mems_send_command(info, command_c))
      {
        dprintf_err("mems_init_link(): Did not see %02X command echo\n", command_c);
        return false;
      }
      if (mems_read_serial(info, &buffer, 1) != 1)
      {
        dprintf_err("mems_init_link(): Did not see null terminator for %02X command\n", command_c);
        return false;
      }
      if (!mems_send_command(info, command_d))
      {
        dprintf_err("mems_init_link(): Did not see %02X command echo\n", command_d);
        return false;
      }

      // Expect four more bytes after the echo of the D0 command byte.
      // Response is 99 00 03 03 for Mini SPi.
      if (mems_read_serial(info, response_buffer, 4) != 4)
      {
        dprintf_err("mems_init_link(): Received fewer bytes than expected after echo of %02X command", command_d);
        return false;
      }

      break;
    }

    case MEMS_Version_2J:
    {
      // Take the line low to wake up the ECU
      const uint32_t break_delay_us = 25000U;
      mems_break_serial(info, break_delay_us);

      // Send the initialisation command
      uint8_t command[] = {0x81, 0x13, 0xF7, 0x81};
      uint8_t response[] = {0xFF, 0xFF, 0xFF};
      if (!(mems_send_command_with_checksum(info, command, sizeof(command)) &&
            mems_read_response_with_checksum(info, response, sizeof(response))))
      {
        dprintf_err("mems_init_link(): Did not receive expected response for first initialisation command\n");
        return false;
      }

      // Wait for an addition 50ms
      info->last_command_us = wait_until_us(info->last_command_us + 50000);

      break;
    }
  }

  return true;
}

/**
 * Locks the mutex used for threadsafe access
 */
bool mems_lock(mems_info* info)
{
#if defined(WIN32)
  if (WaitForSingleObject(info->mutex, INFINITE) != WAIT_OBJECT_0)
    return false;
#else
  pthread_mutex_lock(&info->mutex);
#endif
  return true;
}

/**
 * Releases the mutex used for threadsafe access
 */
void mems_unlock(mems_info* info)
{
#if defined(WIN32)
  ReleaseMutex(info->mutex);
#else
  pthread_mutex_unlock(&info->mutex);
#endif
}

/**
 * Converts the temperature value used by the ECU into degrees Fahrenheit.
 */
uint8_t temperature_value_to_degrees_f(uint8_t val)
{
  uint8_t degrees_c = val - 55;
  return (uint8_t)((float)degrees_c * 1.8 + 32);
}

/**
 * Sends a command to read a frame of data from the ECU, and returns the raw frame.
 */
bool mems_read_raw(mems_info* info, mems_data_frame_80* frame80, mems_data_frame_7d* frame7d)
{
    bool status = false;

    if (mems_lock(info))
    {
      if (mems_send_command(info, MEMS_ReqData80))
      {
        if (mems_read_serial(info, (uint8_t*)(frame80), sizeof(mems_data_frame_80)) == sizeof(mems_data_frame_80))
        {
          status = true;
        }
        else
        {
          dprintf_err("mems_read_raw(): failed to read data frame in response to cmd 0x80\n");
        }
      }
      else
      {
        dprintf_err("mems_read_raw(): failed to send read command 0x80\n");
      }

      if (status)
      {
        if (mems_send_command(info, MEMS_ReqData7D))
        {
          if (!mems_read_serial(info, (uint8_t*)(frame7d), sizeof(mems_data_frame_7d)) == sizeof(mems_data_frame_7d))
          {
            dprintf_err("mems_read_raw(): failed to read data frame in response to cmd 0x7D\n");
            status = false;
          }
        }
        else
        {
          dprintf_err("mems_read_raw(): failed to send read command 0x7D\n");
          status = false;
        }
      }

      mems_unlock(info);
    }

    return status;
}

/**
 * Sends an command to read a frame of data from the ECU, and parses the returned frame.
 */
bool mems_read(mems_info* info, mems_data* data)
{
  bool success = false;
  mems_data_frame_80 dframe80;
  mems_data_frame_7d dframe7d;

  if (mems_read_raw(info, &dframe80, &dframe7d))
  {
    memset(data, 0, sizeof(mems_data));

    data->engine_rpm           = ((uint16_t)dframe80.engine_rpm_hi << 8) | dframe80.engine_rpm_lo;
    data->coolant_temp_f       = temperature_value_to_degrees_f(dframe80.coolant_temp);
    data->ambient_temp_f       = temperature_value_to_degrees_f(dframe80.ambient_temp);
    data->intake_air_temp_f    = temperature_value_to_degrees_f(dframe80.intake_air_temp);
    data->fuel_temp_f          = temperature_value_to_degrees_f(dframe80.fuel_temp);
    data->map_kpa              = dframe80.map_kpa;
    data->battery_voltage      = dframe80.battery_voltage / 10.0;
    data->throttle_pot_voltage = dframe80.throttle_pot * 0.02;
    data->idle_switch          = ((dframe80.idle_switch & 0x10) == 0) ? 0 : 1;
    data->park_neutral_switch  = (dframe80.park_neutral_switch == 0) ? 0 : 1;
    data->fault_codes          = 0;
    data->iac_position         = dframe80.iac_position;
    data->coil_time            = (((uint16_t)dframe80.coil_time_hi << 8) | dframe80.coil_time_lo) * 0.002;
    data->idle_error           = ((uint16_t)dframe80.idle_error_hi << 8) | dframe80.idle_error_lo;
    data->ignition_advance     = (dframe80.ignition_advance * 0.5) - 24.0;
    data->lambda_voltage_mv    = dframe7d.lambda_voltage * 5;
    data->fuel_trim            = dframe7d.fuel_trim;
    data->closed_loop          = dframe7d.closed_loop;
    data->idle_base_pos        = dframe7d.idle_base_pos;

    if (dframe80.dtc0 & 0x01)   // coolant temp sensor fault
      data->fault_codes |= (1 << 0);

    if (dframe80.dtc0 & 0x02)   // intake air temp sensor fault
      data->fault_codes |= (1 << 1);

    if (dframe80.dtc1 & 0x02)   // fuel pump circuit fault
      data->fault_codes |= (1 << 2);

    if (dframe80.dtc1 & 0x80)   // throttle pot circuit fault
      data->fault_codes |= (1 << 3);

    success = true;
  }

  return success;
}

/**
 * Reads the current idle air control motor position.
 */
bool mems_read_iac_position(mems_info* info, uint8_t* position)
{
  bool status = false;

  if (mems_lock(info))
  {
    status = mems_send_command(info, MEMS_GetIACPosition) &&
             (mems_read_serial(info, position, 1) == 1);
    mems_unlock(info);
  }
  return status;
}

/**
 * Repeatedly send command to open or close the idle air control valve until
 * it is in the desired position. The valve does not necessarily move one full
 * step per serial command, depending on the rate at which the commands are
 * issued.
 */
bool mems_move_iac(mems_info* info, uint8_t desired_pos)
{
  bool status = false;
  uint16_t attempts = 0;
  uint8_t current_pos = 0;
  actuator_cmd cmd;

  // read the current IAC position, and only take action
  // if we're not already at the desired point
  if (mems_read_iac_position(info, &current_pos))
  {
    if ((desired_pos < current_pos) ||
        ((desired_pos > current_pos) && (current_pos < IAC_MAXIMUM)))
    {
      cmd = (desired_pos > current_pos) ? MEMS_OpenIAC : MEMS_CloseIAC;

      do {
        status = mems_test_actuator(info, cmd, &current_pos);
        attempts += 1;
      } while (status && (current_pos != desired_pos) && (attempts < 300));
    }
  }

  status = (desired_pos == current_pos);

  return status;
}

/**
 * Sends a command to run an actuator test, and returns the single byte of data.
 */
bool mems_test_actuator(mems_info* info, actuator_cmd cmd, uint8_t* data)
{
  bool status = false;
  uint8_t response = 0x00;

  if (mems_lock(info))
  {
    if (mems_send_command(info, cmd) &&
        (mems_read_serial(info, &response, 1) == 1))
    {
      if (data)
      {
        *data = response;
      }
      status = true;
    }
    mems_unlock(info);
  }
  return status;
}

/**
 * Sends a command to clear any stored fault codes
 */
bool mems_clear_faults(mems_info* info)
{
  bool status = false;
  uint8_t response = 0xFF;

  if (mems_lock(info))
  {
    // send the command and check for one additional byte after the
    // echoed command byte (should be 0x00)
    status = mems_send_command(info, (uint8_t)MEMS_ClearFaults) &&
      (mems_read_serial(info, &response, 1) == 1);
    {
      status = true;
    }
    mems_unlock(info);
  }

  return status;
}

/**
 * Sends a simple heartbeat (ping) command to check connectivity
 */
bool mems_heartbeat(mems_info* info)
{
  bool status = false;

  if (mems_lock(info))
  {
    switch (info->ver)
    {
      case MEMS_Version_16:
      default:
      {
        uint8_t response = 0xFF;
        // send the command and check for one additional byte after the
        // echoed command byte (should be 0x00)
        if (mems_send_command(info, (uint8_t)MEMS_Heartbeat) &&
            (mems_read_serial(info, &response, 1) == 1))
        {
          status = true;
        }
        break;
      }

      case MEMS_Version_2J:
      {
        uint8_t command[] = {0x02, 0x3E, 0x01};
        uint8_t response[] = {0xFF};
        if (mems_send_command_with_checksum(info, command, sizeof(command)) &&
            mems_read_response_with_checksum(info, response, sizeof(response)))
        {
          uint8_t expected_response[] = {0x7E};
          if (memcmp(response, expected_response, sizeof(expected_response)) == 0)
          {
            status = true;
          }
          else
          {
            dprintf_err("mems_heartbeat(): incorrect reponse %02X, expected %02X\n",
              response[0], expected_response[0]);
          }
        }
        break;
      }
    }

    mems_unlock(info);
  }

  return status;
}


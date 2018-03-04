// librosco - a communications library for the Rover MEMS
//
// setup.c: This file contains routines that perform the
//          setup/initialization of the library and the
//          serial port.

#if defined(WIN32) && defined(linux)
#error "Only one of 'WIN32' or 'linux' may be defined."
#endif

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

#if defined(WIN32)
  #include <windows.h>
#else
  #include <string.h>
  #include <termios.h>
  #include <arpa/inet.h>
#endif

#include "rosco.h"
#include "rosco_internal.h"
#include "rosco_version.h"

/**
 * Sets initial values in the state-info struct.
 * Note that this routine does not actually open the serial port or attempt
 * to connect to the ECU; that requires mems_connect().
 * @param info 
 */
void mems_init(mems_info *info, mems_ver ver)
{
    info->ver = ver;
    info->ft = 0;
#if defined(WIN32)
    info->mutex = CreateMutex(NULL, TRUE, NULL);
#else
    pthread_mutex_init(&info->mutex, NULL);
#endif
    info ->last_command_us = 0;
}

/**
 * Disconnects (if necessary) and closes the mutex handle.
 * @param info State information for the current connection.
 */
void mems_cleanup(mems_info *info)
{
    if (mems_is_connected(info))
    {
        FT_Close(info->ft);
        info->ft = 0;
    }
#if defined(WIN32)
    CloseHandle(info->mutex);
#else
    pthread_mutex_destroy(&info->mutex);
#endif
}

/**
 * Returns version information for this build of the library.
 * @return Version of this build of librosco
 */
librosco_version mems_get_lib_version()
{
    librosco_version ver;

    ver.major = LIBROSCO_VER_MAJOR;
    ver.minor = LIBROSCO_VER_MINOR;
    ver.patch = LIBROSCO_VER_PATCH;

    return ver;
}

/**
 * Closes the serial device.
 * @param info State information for the current connection.
 */
void mems_disconnect(mems_info *info)
{
#if defined(WIN32)
    if (WaitForSingleObject(info->mutex, INFINITE) == WAIT_OBJECT_0)
    {
        if (mems_is_connected(info))
        {
            FT_Close(info->ft);
            info->ft = 0;
        }

        ReleaseMutex(info->mutex);
    }
#else
    pthread_mutex_lock(&info->mutex);

    if (mems_is_connected(info))
    {
        FT_Close(info->ft);
        info->ft = 0;
    }

    pthread_mutex_unlock(&info->mutex);
#endif
}

/**
 * Opens the FTDI device (or returns with success if it is already open.)
 * @param info State information for the current connection.
 * @return True if the serial device was successfully opened and its
 *   baud rate was set; false otherwise.
 */
bool mems_connect(mems_info *info)
{
    bool result = false;

#if defined(WIN32)
    if (WaitForSingleObject(info->mutex, INFINITE) == WAIT_OBJECT_0)
    {
        result = mems_is_connected(info) || mems_openserial(info);
        ReleaseMutex(info->mutex);
    }
#else // Linux/Unix
    pthread_mutex_lock(&info->mutex);
    result = mems_is_connected(info) || mems_openserial(info);
    pthread_mutex_unlock(&info->mutex);
#endif

    return result;
}

/**
 * Opens the FTDI device for the USB<->TTL/serial converter and sets the
 * parameters for the link to match those on the MEMS ECU.
 *
 * This assumes there is only a single FTDI device available as it simply
 * opens the first one available.
 *
 * @return True if the open/setup was successful, false otherwise
 */
bool mems_openserial(mems_info *info)
{
    bool retVal = false;
    FT_STATUS status;

    status = FT_Open(0, &info->ft);
    if (status == FT_OK)
    {
        status = FT_SetDataCharacteristics(info->ft, FT_BITS_8, FT_STOP_BITS_1, FT_PARITY_NONE);
        if (status == FT_OK)
        {
            switch (info->ver)
            {
                case MEMS_Version_16:
                default:
                {
                    status = FT_SetBaudRate(info->ft, 9600);
                    break;
                }

                case MEMS_Version_2J:
                {
                    status = FT_SetBaudRate(info->ft, 10400);
                    break;
                }
            }

            if (status == FT_OK)
            {
                status = FT_SetTimeouts(info->ft, 100, 100);
                if (status == FT_OK)
                {
                    status = FT_SetFlowControl(info->ft, FT_FLOW_NONE, 0, 0);
                    if (status == FT_OK)
                    {
                        retVal = true;
                    }
                    else
                    {
                        dprintf_err("mems_openserial(): Failed to configure flow control: status = %d\n", status);
                    }
                }
                else
                {
                    dprintf_err("mems_openserial(): Failed to set timeouts: status = %d\n", status);
                }
            }
            else
            {
                dprintf_err("mems_openserial(): Failed to set BAUD rate: status = %d\n", status);
            }
        }
        else
        {
            dprintf_err("mems_openserial(): Failed to set serial characteristics: status = %d\n", status);
        }

        if (status != FT_OK)
        {
            FT_Close(info->ft);
        }
    }
    else
    {
        dprintf_err("mems_openserial(): Failed to open FTDI device: status = %d\n", status);
    }

    return retVal;
}

/**
 * Checks the file descriptor for the serial device to determine if it has
 * already been opened.
 * @return True if the serial device is open; false otherwise.
 */
bool mems_is_connected(mems_info* info)
{
    return (info->ft != 0);
}


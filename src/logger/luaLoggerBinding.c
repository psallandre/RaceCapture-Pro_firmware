/*
 * Race Capture Firmware
 *
 * Copyright (C) 2016 Autosport Labs
 *
 * This file is part of the Race Capture firmware suite
 *
 * This is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details. You should
 * have received a copy of the GNU General Public License along with
 * this code. If not, see <http://www.gnu.org/licenses/>.
 */

#include "ADC.h"
#include "CAN.h"
#include "FreeRTOS.h"
#include "GPIO.h"
#include "led.h"
#include "OBD2.h"
#include "PWM.h"
#include "channel_config.h"
#include "dateTime.h"
#include "gps.h"
#include "imu.h"
#include "luaBaseBinding.h"
#include "lap_stats.h"
#include "lauxlib.h"
#include "logger.h"
#include "loggerConfig.h"
#include "loggerData.h"
#include "loggerSampleData.h"
#include "loggerTaskEx.h"
#include "luaLoggerBinding.h"
#include "luaScript.h"
#include "luaTask.h"
#include "mod_string.h"
#include "modp_numtoa.h"
#include "printk.h"
#include "queue.h"
#include "semphr.h"
#include "serial.h"
#include "task.h"
#include "timer.h"
#include "virtual_channel.h"

#define TEMP_BUFFER_LEN 		200
#define DEFAULT_CAN_TIMEOUT 	100
#define DEFAULT_SERIAL_TIMEOUT	100
#define LUA_DEFAULT_SERIAL_PORT SERIAL_AUX
#define LUA_DEFAULT_SERIAL_BAUD 115200
#define LUA_DEFAULT_SERIAL_BITS 8
#define LUA_DEFAULT_SERIAL_PARITY 0
#define LUA_DEFAULT_SERIAL_STOP_BITS 1

char g_tempBuffer[TEMP_BUFFER_LEN];
static int lua_get_virtual_channel(lua_State *ls);
static int lua_set_led(lua_State *ls);

static int lua_get_uptime(lua_State *L)
{
        lua_pushinteger(L, (int) getUptime());
        return 1;
}

static int lua_get_date_time(lua_State *L)
{
        const GpsSample sample = getGpsSample();

        /*
         * Have to use DateTime because LUA only uses floats and ints.
         * Time is a uint64t value so can't just use it directly.  Hence
         * this.  If folks want a monotonically increasing value, they
         * need to use getUptime call.
         */
        DateTime dt;
        getDateTimeFromEpochMillis(&dt, sample.time);

        lua_pushinteger(L, (int) dt.year);
        lua_pushinteger(L, (int) dt.month);
        lua_pushinteger(L, (int) dt.day);
        lua_pushinteger(L, (int) dt.hour);
        lua_pushinteger(L, (int) dt.minute);
        lua_pushinteger(L, (int) dt.second);
        lua_pushinteger(L, (int) dt.millisecond);

        return 7;
}

static int lua_get_gps_altitude(lua_State *L)
{
        lua_pushnumber(L, getAltitude());
        return 1;
}

void registerLuaLoggerBindings(lua_State *L)
{

    //Read / control inputs and outputs
    lua_registerlight(L,"getGpio",Lua_GetGPIO);
    lua_registerlight(L,"setGpio",Lua_SetGPIO);
    lua_registerlight(L,"getButton",Lua_GetButton);

    lua_registerlight(L,"setPwmDutyCycle",Lua_SetPWMDutyCycle);
    lua_registerlight(L,"setPwmPeriod",Lua_SetPWMPeriod);
    lua_registerlight(L,"setAnalogOut",Lua_SetAnalogOut);
    lua_registerlight(L,"setPwmClockFreq",Lua_SetPWMClockFrequency);
    lua_registerlight(L,"getPwmClockFreq",Lua_GetPWMClockFrequency);

    lua_registerlight(L,"getTimerRpm",Lua_GetRPM);
    lua_registerlight(L,"getTimerPeriodMs",Lua_GetPeriodMs);
    lua_registerlight(L,"getTimerFreq",Lua_GetFrequency);
    lua_registerlight(L,"getTimerRaw",Lua_GetTimerRaw);
    lua_registerlight(L,"resetTimerCount",Lua_ResetTimerCount);
    lua_registerlight(L,"getTimerCount",Lua_GetTimerCount);

    lua_registerlight(L,"getAnalog",Lua_GetAnalog);

    lua_registerlight(L,"getImu",Lua_ReadImu);
    lua_registerlight(L,"getImuRaw",Lua_ReadImuRaw);

    lua_registerlight(L, "getGpsSats", Lua_GetGPSSatellites);
    lua_registerlight(L, "getGpsPos", Lua_GetGPSPosition);
    lua_registerlight(L, "getGpsSpeed", Lua_GetGPSSpeed);
    lua_registerlight(L, "getGpsQuality", Lua_GetGPSQuality);
    lua_registerlight(L, "getGpsDist", Lua_GetGPSDistance);
    lua_registerlight(L, "getGpsAltitude", lua_get_gps_altitude);

    lua_registerlight(L, "getLapCount", Lua_GetLapCount);
    lua_registerlight(L, "getLapTime", Lua_GetLapTime);
    lua_registerlight(L, "getGpsSec", Lua_GetGpsSecondsSinceFirstFix);
    lua_registerlight(L, "getAtStartFinish",Lua_GetGPSAtStartFinish);

    lua_registerlight(L, "getTickCount", Lua_GetTickCount);
    lua_registerlight(L, "getTicksPerSecond", Lua_GetTicksPerSecond);

    lua_registerlight(L, "initCAN", Lua_InitCAN);
    lua_registerlight(L, "txCAN", Lua_SendCANMessage);
    lua_registerlight(L, "rxCAN", Lua_ReceiveCANMessage);
    lua_registerlight(L, "setCANfilter", Lua_SetCANFilter);
    lua_registerlight(L, "readOBD2", Lua_ReadOBD2);

    lua_registerlight(L,"startLogging",Lua_StartLogging);
    lua_registerlight(L,"stopLogging",Lua_StopLogging);
    lua_registerlight(L,"isLogging" , Lua_IsLogging);

    lua_registerlight(L,"setLed", lua_set_led);

    //Serial API
    lua_registerlight(L,"initSer", Lua_InitSerial);
    lua_registerlight(L,"readCSer", Lua_ReadSerialChar);
    lua_registerlight(L,"readSer", Lua_ReadSerialLine);
    lua_registerlight(L,"writeCSer", Lua_WriteSerialChar);
    lua_registerlight(L,"writeSer", Lua_WriteSerialLine);

    //Logger configuration editing
    lua_registerlight(L,"flashLoggerCfg", Lua_FlashLoggerConfig);

    lua_registerlight(L,"calibrateImuZero",Lua_CalibrateImuZero);

    lua_registerlight(L,"setBgStream", Lua_SetBackgroundStreaming);
    lua_registerlight(L,"getBgStream", Lua_GetBackgroundStreaming);

    lua_registerlight(L, "addChannel", Lua_AddVirtualChannel);
    lua_registerlight(L, "getChannel", lua_get_virtual_channel);
    lua_registerlight(L, "setChannel", Lua_SetVirtualChannelValue);

    /* Timing info */
    lua_registerlight(L, "getUptime", lua_get_uptime);
    lua_registerlight(L, "getDateTime", lua_get_date_time);

    //EvoX
    lua_registerlight(L, "addDefaultEngineChannels", lua_AddDefaultEngineChannels);
    lua_registerlight(L, "requestAndReadCAN", Lua_requestAndReadCAN);
    lua_registerlight(L, "requestAndReadCANAndDecodeTephraV3", Lua_requestAndReadCANAndDecodeTephraV3);
    lua_registerlight(L, "setRearO2ChannelId", Lua_set_RearO2_ChannelId);
}

////////////////////////////////////////////////////

int Lua_SetBackgroundStreaming(lua_State *L)
{
    if (lua_gettop(L) >= 1) {
        getWorkingLoggerConfig()->ConnectivityConfigs.telemetryConfig.backgroundStreaming = (lua_tointeger(L,1) == 1);
    }
    return 0;
}

int Lua_GetBackgroundStreaming(lua_State *L)
{
    lua_pushinteger(L, getWorkingLoggerConfig()->ConnectivityConfigs.telemetryConfig.backgroundStreaming == 1);
    return 1;
}

int Lua_CalibrateImuZero(lua_State *L)
{
    imu_calibrate_zero();
    return 0;
}

int Lua_GetGPSAtStartFinish(lua_State *L)
{
    lua_pushinteger(L,getAtStartFinish());
    return 1;
}

int Lua_GetAtSplit(lua_State *L)
{
    lua_pushinteger(L,getAtSector());
    return 1;
}


int Lua_SetPWMClockFrequency(lua_State *L)
{
    if (lua_gettop(L) >= 1) {
        LoggerConfig *loggerConfig = getWorkingLoggerConfig();

        uint16_t clock_frequency = filterPwmClockFrequency(lua_tointeger(L,1));
        loggerConfig->PWMClockFrequency = clock_frequency;
        PWM_set_clock_frequency(clock_frequency);
    }
    return 0;
}

int Lua_GetPWMClockFrequency(lua_State *L)
{
    lua_pushinteger(L,getWorkingLoggerConfig()->PWMClockFrequency);
    return 1;
}

int Lua_GetAnalog(lua_State *L)
{
    float analogValue = -1;
    if (lua_gettop(L) >= 1) {
        unsigned int channel = (unsigned int)lua_tointeger(L,1);
        ADCConfig *ac = getADCConfigChannel(lua_tointeger(L,1));
        if (NULL != ac) {
            float adcRaw = ADC_read(channel);
            switch(ac->scalingMode) {
            case SCALING_MODE_RAW:
                analogValue = adcRaw;
                break;
            case SCALING_MODE_LINEAR:
                analogValue = (ac->linearScaling * (float)adcRaw);
                break;
            case SCALING_MODE_MAP:
                analogValue = get_mapped_value((float)adcRaw,&(ac->scalingMap));
                break;
            }
        }
    }
    lua_pushnumber(L, analogValue);
    return 1;
}

static int luaToTimerValues(lua_State *L, uint8_t *pulsePerRevolution, size_t *channel)
{
    int result = 0;
    if (lua_gettop(L) >=  1) {
        size_t requestedChannel = lua_tointeger(L,1);
        TimerConfig *c = get_timer_config(requestedChannel);
        if (NULL != c) {
            *pulsePerRevolution = c->pulsePerRevolution;
            *channel = requestedChannel;
            result = 1;
        }
    }
    return result;
}

int Lua_GetRPM(lua_State *L)
{
    uint8_t pulsePerRevolution;
    size_t channel;
    int result = 0;
    if (luaToTimerValues(L, &pulsePerRevolution, &channel)) {
        int rpm = timer_get_rpm(channel) / pulsePerRevolution;
        lua_pushinteger(L, rpm);
        result = 1;
    }
    return result;
}


int Lua_GetPeriodMs(lua_State *L)
{
    uint8_t pulsePerRevolution;
    size_t channel;
    int result = 0;
    if (luaToTimerValues(L, &pulsePerRevolution, &channel)) {
        int period = timer_get_ms(channel) * pulsePerRevolution;
        lua_pushinteger(L, period);
        result = 1;
    }
    return result;
}

int Lua_GetFrequency(lua_State *L)
{
    uint8_t pulsePerRevolution;
    size_t channel;
    int result = 0;
    if (luaToTimerValues(L, &pulsePerRevolution, &channel)) {
        int hz = timer_get_hz(channel) / pulsePerRevolution;
        lua_pushinteger(L, hz);
        result = 1;
    }
    return result;
}

int Lua_GetTimerRaw(lua_State *L)
{
    int result = -1;
    if (lua_gettop(L) >= 1) {
        int channel = lua_tointeger(L,1);
        result = timer_get_raw(channel);
    }
    lua_pushinteger(L,result);
    return 1;
}

int Lua_ResetTimerCount(lua_State *L)
{
    if (lua_gettop(L) >= 1) {
        timer_reset_count(lua_tointeger(L,1));
    }
    return 0;
}

int Lua_GetTimerCount(lua_State *L)
{
    int result = -1;
    if (lua_gettop(L) >= 1) {
        int channel = lua_tointeger(L,1);
        result = timer_get_count(channel);
    }
    lua_pushinteger(L,result);
    return 1;
}

int Lua_GetButton(lua_State *L)
{
    unsigned int pushbutton = GPIO_is_button_pressed();
    lua_pushinteger(L,(pushbutton == 0));
    return 1;
}

/**
 * Initializes the specified serial port
 * Lua Params:
 * port - the serial port to initialize
 *        (SERIAL_USB, SERIAL_GPS, SERIAL_TELEMETRY, SERIAL_WIRELESS, SERIAL_AUX) (defaults to SERIAL_AUX)
 * baud - Baud Rate (defaults to 115200)
 * bits - Number of bit in the message (8 or 7) (defaults to 8)
 * parity - (1 = Even Parity, 2 = Odd Parity, 0 = No Parity) (defaults to No Parity)
 * stopBits - number of stop bits (1 or 2) (defaults to 1)
 *
 * Lua Returns:
 * 1 if successful; -1 if parameter error
 */
int Lua_InitSerial(lua_State *L)
{
    int params = lua_gettop(L);

    serial_id_t port = LUA_DEFAULT_SERIAL_PORT;
    uint32_t baud = LUA_DEFAULT_SERIAL_BAUD;
    uint8_t bits  = LUA_DEFAULT_SERIAL_BITS;
    uint8_t parity = LUA_DEFAULT_SERIAL_PARITY;
    uint8_t stop_bits = LUA_DEFAULT_SERIAL_STOP_BITS;

    switch(params) {
    case 5:
        stop_bits = lua_tointeger(L, 5);
    case 4:
        parity = lua_tointeger(L, 4);
    case 3:
        bits = lua_tointeger(L, 3);
    case 2:
        baud = lua_tointeger(L, 2);
    case 1:
        port = lua_tointeger(L, 1);
    case 0:
        configure_serial(port, bits, parity, stop_bits, baud);
        lua_pushnumber(L, 1);
        break;
    default:
        lua_pushnumber(L, -1);
        break;
    }
    return 1;
}

/**
 * Read a character from the specified serial port
 * Lua Params:
 * port - the serial port to initialize
 *        (SERIAL_USB, SERIAL_GPS, SERIAL_TELEMETRY, SERIAL_WIRELESS, SERIAL_AUX)
 * timeout - the read timeout, in ms.
 *
 * Lua Returns:
 * the character read, or nil if no characters received (receive timeout)
 *
 */
int Lua_ReadSerialChar(lua_State *L)
{
    int params = lua_gettop(L);
    if (!params)
        return 0;

    serial_id_t port = lua_tointeger(L,1);
    size_t timeout = params >= 2 ? lua_tointeger(L, 2) : DEFAULT_SERIAL_TIMEOUT;
    Serial *serial = get_serial(port);
    if (serial) {
        char c;
        if (serial->get_c_wait(&c, timeout)) {
            lua_pushnumber(L, c);
            return 1;
        }
    }
    return 0;
}

/**
 * Read a single newline terminated line from the specified serial port
 * Lua Params:
 * port - the serial port to initialize
 *        (SERIAL_USB, SERIAL_GPS, SERIAL_TELEMETRY, SERIAL_WIRELESS, SERIAL_AUX)
 * timeout - the read timeout, in ms.
 *
 * Lua Returns:
 * the character read, or nil if no characters received (receive timeout)
 *
 */
int Lua_ReadSerialLine(lua_State *L)
{
    size_t params = lua_gettop(L);
    if (!params)
        return 0;

    int serialPort = lua_tointeger(L,1);
    size_t timeout = params >= 2 ? lua_tointeger(L, 2) : DEFAULT_SERIAL_TIMEOUT;
    Serial *serial = get_serial(serialPort);
    if (serial) {
        serial->get_line_wait(g_tempBuffer, TEMP_BUFFER_LEN, timeout);
        lua_pushstring(L,g_tempBuffer);
        return 1;
    }
    return 0;
}

/**
 * Writes the specified line to the serial port.
 * The call will block until all characters are written.
 *
 * Lua Params:
 * port - the serial port to write
 *        (SERIAL_USB, SERIAL_GPS, SERIAL_TELEMETRY, SERIAL_WIRELESS, SERIAL_AUX)
 * line - the string to write. A newline will automatically be added at the end.
 *
 * Lua Returns:
 * no return values (nil)
 *
 */
int Lua_WriteSerialLine(lua_State *L)
{
    if (lua_gettop(L) >= 2) {
        int serialPort = lua_tointeger(L,1);
        Serial *serial = get_serial(serialPort);
        if (serial) {
            const char * data = lua_tostring(L, 2);
            serial->put_s(data);
            serial->put_s("\n");
        }
    }
    return 0;
}

/**
 * Writes the specified character to the serial port.
 * The call will block until the character is written.
 *
 * Lua Params:
 * port - the serial port to write
 *        (SERIAL_USB, SERIAL_GPS, SERIAL_TELEMETRY, SERIAL_WIRELESS, SERIAL_AUX)
 * char - the character to write.
 *
 * Lua Returns:
 * no return values (nil)
 *
 */
int Lua_WriteSerialChar(lua_State *L)
{
    if (lua_gettop(L) >= 2) {
        int serialPort = lua_tointeger(L,1);
        Serial *serial = get_serial(serialPort);
        if (serial) {
            char c = (char)lua_tonumber(L, 2);
            serial->put_c((char)c);
        }
    }
    return 0;
}

int Lua_GetGPIO(lua_State *L)
{
    unsigned int state = 0;
    if (lua_gettop(L) >= 1) {
        unsigned int channel = (unsigned int)lua_tointeger(L,1);
        state = GPIO_get(channel);
    }
    lua_pushinteger(L, state);
    return 1;
}

int Lua_SetGPIO(lua_State *L)
{
    if (lua_gettop(L) >=2) {
        unsigned int channel = (unsigned int)lua_tointeger(L,1);
        unsigned int state = ((unsigned int)lua_tointeger(L,2) == 1);
        GPIO_set(channel, state);
    }
    return 0;
}

int Lua_GetGPSSatellites(lua_State *L)
{
    lua_pushnumber(L, GPS_getSatellitesUsedForPosition());
    return 1;
}

int Lua_GetGPSPosition(lua_State *L)
{
    lua_pushnumber(L, GPS_getLatitude());
    lua_pushnumber(L, GPS_getLongitude());
    return 2;
}

int Lua_GetGPSSpeed(lua_State *L)
{
    lua_pushnumber(L,getGpsSpeedInMph());
    return 1;
}

int Lua_GetGPSQuality(lua_State *L)
{
    lua_pushnumber(L,GPS_getQuality());
    return 1;
}

int Lua_GetGPSDistance(lua_State *L)
{
    lua_pushnumber(L, getLapDistanceInMiles());
    return 1;
}

int Lua_GetLapTime(lua_State *L)
{
    // XXX: TIME_HACK.  Convert to millis.
    lua_pushnumber(L, tinyMillisToMinutes(getLastLapTime()));
    return 1;
}

int Lua_GetLapCount(lua_State *L)
{
    lua_pushinteger(L,getLapCount());
    return 1;
}

int Lua_GetGpsSecondsSinceFirstFix(lua_State *L)
{
    const tiny_millis_t millis = getMillisSinceFirstFix();
    // XXX: TIME_HACK.  Convert to millis.
    lua_pushnumber(L, tinyMillisToSeconds(millis));
    return 1;
}

int Lua_GetTickCount(lua_State *L)
{
    const float f = (float) xTaskGetTickCount();
    lua_pushnumber(L, f);
    return 1;
}

int Lua_GetTicksPerSecond(lua_State *L)
{
    const float f = (float) configTICK_RATE_HZ;
    lua_pushnumber(L, f);
    return 1;
}

int Lua_ReadImu(lua_State *L)
{
    if (lua_gettop(L) >= 1) {
        unsigned int channel = (unsigned int)lua_tointeger(L,1);
        if (channel >= 0 && channel < CONFIG_IMU_CHANNELS) {
            ImuConfig *ac = &getWorkingLoggerConfig()->ImuConfigs[channel];
            float value = imu_read_value(channel,ac);
            lua_pushnumber(L, value);
            return 1;
        }
    }
    return 0;
}

int Lua_ReadImuRaw(lua_State *L)
{
    if (lua_gettop(L) >= 1) {
        unsigned int channel = (unsigned int)lua_tointeger(L,1);
        if (channel >= 0 && channel <= CONFIG_IMU_CHANNELS) {
            int value =  imu_read(channel);
            lua_pushinteger(L,value);
            return 1;
        }
    }
    return 0;
}


int Lua_SetPWMDutyCycle(lua_State *L)
{
    if (lua_gettop(L) >= 2) {
        unsigned int channel = (unsigned int)lua_tointeger(L,1);
        unsigned int dutyCycleRaw = (unsigned int)lua_tointeger(L,2);
        if (channel >= 0 && channel < CONFIG_PWM_CHANNELS) {
            PWM_set_duty_cycle( channel, (unsigned short)dutyCycleRaw);
        }
    }
    return 0;
}

int Lua_SetPWMPeriod(lua_State *L)
{
    if (lua_gettop(L) >= 2) {
        unsigned int channel = (unsigned int)lua_tointeger(L,1);
        unsigned int periodRaw = (unsigned int)lua_tointeger(L,2);
        if (channel >= 0 && channel < CONFIG_PWM_CHANNELS) {
            PWM_channel_set_period(channel, (unsigned short)periodRaw);
        }
    }
    return 0;
}

int Lua_SetAnalogOut(lua_State *L)
{
    if (lua_gettop(L) >= 2) {
        PWMConfig *c = getPwmConfigChannel(lua_tointeger(L,1));
        if (NULL != c) {
            unsigned int channel = (unsigned int)lua_tointeger(L,1);
            if (channel >= 0 && channel < CONFIG_PWM_CHANNELS) {
                float dutyCycle = (float)lua_tonumber(L,2) / PWM_VOLTAGE_SCALING;
                PWM_set_duty_cycle(channel,(unsigned short)dutyCycle);
            }
        }
    }
    return 0;
}

int Lua_InitCAN(lua_State *L)
{
    if (lua_gettop(L) >= 2) {
        size_t port = (size_t)lua_tointeger(L, 1);
        uint32_t baud = lua_tointeger(L, 2);
        int rc = CAN_init_port(port, baud);
        lua_pushinteger(L, rc);
        return 1;
    } else {
        return 0;
    }
}

int Lua_SetCANFilter(lua_State *L)
{
    if (lua_gettop(L) >= 5) {
        uint8_t channel = lua_tointeger(L, 1);
        uint8_t id = lua_tointeger(L, 2);
        uint8_t extended = lua_tointeger(L, 3);
        uint32_t filter = lua_tointeger(L, 4);
        uint32_t mask = lua_tointeger(L, 5);
        int rc = CAN_set_filter(channel, id, extended, filter, mask);
        lua_pushinteger(L, rc);
        return 1;
    }
    return 0;
}

int Lua_SendCANMessage(lua_State *L)
{
    size_t args = lua_gettop(L);
    if (args >= 4) {
        CAN_msg msg;
        uint8_t channel = (uint8_t)lua_tointeger(L, 1);
        msg.addressValue = (unsigned int)lua_tointeger(L, 2);
        msg.isExtendedAddress = lua_tointeger(L, 3);
        int size = luaL_getn(L, 4);
        size_t timeout = args >= 5 ? lua_tointeger(L, 5) : DEFAULT_CAN_TIMEOUT;
        if (size <= CAN_MSG_SIZE) {
            for (int i = 1; i <= size; i++) {
                lua_pushnumber(L,i);
                lua_gettable(L, 4);
                int val = lua_tonumber(L, -1);
                msg.data[i - 1] = val;
                lua_pop(L, 1);
            }
        }
        msg.dataLength = size;
        int rc = CAN_tx_msg(channel, &msg, timeout);
        lua_pushinteger(L, rc);
        return 1;
    }
    return 0;
}

int Lua_ReceiveCANMessage(lua_State *L)
{
    size_t timeout = DEFAULT_CAN_TIMEOUT;
    if (lua_gettop(L) >= 1) {
        uint8_t channel = (uint8_t)lua_tointeger(L, 1);
        if (lua_gettop(L) >= 2) timeout = lua_tointeger(L, 2);

        CAN_msg msg;
        int rc = CAN_rx_msg(channel, &msg, timeout);
        if (rc == 1) {
            lua_pushinteger(L, msg.addressValue);
            lua_pushinteger(L, msg.isExtendedAddress);
            lua_newtable(L);
            for (int i = 1; i <= msg.dataLength; i++) {
                lua_pushnumber(L, i);
                lua_pushnumber(L, msg.data[i - 1]);
                lua_rawset(L, -3);
            }
            return 3;
        }
        return 0;
    }
    lua_pushnumber(L, -1);
    return 1;
}

int Lua_ReadOBD2(lua_State *L)
{
    if (lua_gettop(L) >= 1) {
        unsigned char pid = (unsigned char)lua_tointeger(L, 1);
        int value;
        if (OBD2_request_PID(pid, &value, OBD2_PID_DEFAULT_TIMEOUT_MS)) {
            lua_pushnumber(L, value);
            return 1;
        }
    }
    return 0;
}

int Lua_StartLogging(lua_State *L)
{
    startLogging();
    return 0;
}

int Lua_StopLogging(lua_State *L)
{
    stopLogging();
    return 0;
}

int Lua_IsLogging(lua_State *L)
{
    lua_pushinteger(L, logging_is_active());
    return 1;
}

static int lua_set_led(lua_State *ls)
{
        if (lua_gettop(ls) != 2)
                return incorrect_arguments(ls);

        const bool is_num = lua_isnumber(ls, 1);
        const bool is_str = lua_isstring(ls, 1);
        if (!is_num && !is_str)
                return luaL_error(ls, "This method only accepts LED names "
                                  "or LED IDs");

        /*
         * Numbers can be passed in as 123 or "123" in lua.  So handle
         * that case first
         */
        const bool on = lua_toboolean(ls, 2);
        bool res;
        if (is_num) {
                res = led_set_index(lua_tointeger(ls, 1), on);
        } else {
                const enum led led = get_led_enum(lua_tostring(ls, 1));
                res = led_set(led, on);
        }

        lua_pushboolean(ls, res);
        return 1;
}


int Lua_FlashLoggerConfig(lua_State *L)
{
    enum memory_flash_result_t result = flashLoggerConfig();
    lua_pushinteger(L,result);
    return 1;
}

int Lua_AddVirtualChannel(lua_State *L)
{
        const size_t args = lua_gettop(L);
        if (args < 2 || args > 6)
                return incorrect_arguments(L);

        ChannelConfig cc;
        channel_config_defaults(&cc);

        switch (validate_channel_config_label(lua_tostring(L, 1))) {
        case CHAN_CFG_STATUS_OK:
                strcpy(cc.label, lua_tostring(L, 1));
                break;
        case CHAN_CFG_STATUS_NO_LABEL:
                luaL_error(L, "Label is empty");
        case CHAN_CFG_STATUS_LONG_LABEL:
                luaL_error(L, "Label is too long");
        default:
                goto bug;
        }

        cc.sampleRate = encodeSampleRate((unsigned short) lua_tointeger(L, 2));
        if (SAMPLE_DISABLED == cc.sampleRate)
                return luaL_error(L, "Unsupported sample rate");

        switch(args) {
        case 6:
                switch (validate_channel_config_units(lua_tostring(L, 6))) {
                case CHAN_CFG_STATUS_OK:
                        strcpy(cc.units, lua_tostring(L, 6));
                        break;
                case CHAN_CFG_STATUS_NO_UNITS:
                        return luaL_error(L, "Units is empty");
                case CHAN_CFG_STATUS_LONG_UNITS:
                        return luaL_error(L, "Units is too long");
                default:
                        goto bug;
                }
        case 5:
                cc.max = lua_tointeger(L, 5);
        case 4:
                cc.min = lua_tointeger(L, 4);
        case 3:
                cc.precision = lua_tointeger(L, 3);
        }

        switch(validate_channel_config(&cc)) {
        case CHAN_CFG_STATUS_OK:
                break;
        case CHAN_CFG_STATUS_MAX_LT_MIN:
                return luaL_error(L, "Max is less than min");
        default:
                goto bug;
        }

        const int chan_id = create_virtual_channel(cc);
        if (INVALID_VIRTUAL_CHANNEL == chan_id)
                return luaL_error(L, "Unable to create channel. "
                                     "Maximum channels reached.");

        lua_pushinteger(L, chan_id);
        return 1;

bug:
        return luaL_error(L, "BUG. Should never get here.  "
                             "Please inform AutosportLabs");
}

static int lua_get_virtual_channel(lua_State *ls)
{
    if (lua_gettop(ls) != 1)
            return incorrect_arguments(ls);

    const bool is_num = lua_isnumber(ls, 1);
    const bool is_str = lua_isstring(ls, 1);
    if (!is_num && !is_str)
            return luaL_error(ls, "This method only accepts channel names "
                              "or channel IDs");

    /*
     * Numbers can be passed in as 123 or "123" in lua.  So handle
     * that case first
     */
    VirtualChannel *vc;
    if (is_num) {
            vc = get_virtual_channel(lua_tointeger(ls, 1));
    } else {
            const int idx = find_virtual_channel(lua_tostring(ls, 1));
            vc = get_virtual_channel(idx);
    }

    if (!vc)
            return luaL_error(ls, "Virtual channel not found!");

    lua_pushnumber(ls, vc->currentValue);
    return 1;
}

int Lua_SetVirtualChannelValue(lua_State *L)
{
        const size_t args = lua_gettop(L);
        if (args != 2)
                return incorrect_arguments(L);

        const int id = lua_tointeger(L, 1);
        const float value = lua_tonumber(L, 2);
        set_virtual_channel_value(id, value);

        return 0;
}

#define CHANNEL_COUNT 31
static int chan_id[CHANNEL_COUNT];
static int chLoad        ;
static int chRPM         ;
static int chBoost       ;
static int chIPW         ;
static int chTiming      ;
static int chKnockSum    ;
static int chTPS         ;
static int chAPP         ;
static int chMAP         ;
static int chBaro        ;
static int chWGDC        ;
static int chWGDCCorr    ;
static int chECT         ;
static int chIAT         ;
static int chMAT         ;
static int chSpeed       ;
static int chKnockBase   ;
static int chKnockFilter ;
static int chSTFT        ;
static int chLTFT        ;
static int chLTFTIdle    ;
static int chLTFTCruise  ;
static int chMIVInAct    ;
static int chMIVExAct    ;
static int chMAFVolt     ;
static int chLoadTiming  ;
static int chLoadMAP     ;
static int chLoadIMAP    ;
static int chLoadMAF     ;
static int chLoadChosen  ;
static int chEthanol     ;
int lua_AddDefaultEngineChannels(lua_State *L){
    if (lua_gettop(L) != 2)
        return incorrect_arguments(L);
    unsigned short sampleRateLow = encodeSampleRate((unsigned short) lua_tointeger(L, 1));
    unsigned short sampleRateHigh = encodeSampleRate((unsigned short) lua_tointeger(L, 2));

    //label,units,min,max,sampleRate,precision,flags;
    ChannelConfig cc[CHANNEL_COUNT] ={
        {"Load"       , "Load"  , 0    , 500  , sampleRateHigh, 1, 0},
        {"RPM"        , "rpm"   , 0    , 10000, sampleRateHigh, 0, 0},
        {"Boost"      , "Psig"  , -14.5, 43.5 , sampleRateHigh, 2, 0},
        {"IPW"        , "ms"    , 0    , 25.5 , sampleRateHigh, 3, 0},
        {"Timing"     , "deg"   , -20  , 107  , sampleRateHigh, 0, 0},
        {"KnockSum"   , "counts", 0    , 63   , sampleRateHigh, 0, 0},
        {"TPS"        , "%"     , 0    , 100  , sampleRateHigh, 1, 0},
        {"APP"        , "%"     , 0    , 100  , sampleRateHigh, 1, 0},
        {"MAP"        , "Psia"  , 0    , 58   , sampleRateHigh, 2, 0},
        {"Baro"       , "Psia"  , 6.52 , 15.7 , sampleRateLow , 2, 0},
        {"WGDC"       , "%"     , 0    , 100  , sampleRateHigh, 1, 0},
        {"WGDCCorr"   , "%"     , -64  , 64   , sampleRateHigh, 1, 0},
        {"ECT"        , "C"     , -40  , 200  , sampleRateLow , 0, 0},
        {"IAT"        , "C"     , -40  , 200  , sampleRateLow , 0, 0},
        {"MAT"        , "C"     , -40  , 200  , sampleRateLow , 0, 0},
        {"Speed_ECU"  , "Km/h"  , 0    , 300  , sampleRateHigh, 1, 0},
        {"KnockBase"  , "counts", 0    , 255  , sampleRateHigh, 0, 0},
        {"KnockFilter", "counts", 0    , 255  , sampleRateHigh, 0, 0},
        {"STFT"       , "%"     , -30  , 30   , sampleRateHigh, 2, 0},
        {"LTFT"       , "%"     , -30  , 30   , sampleRateHigh, 2, 0},
        {"LTFTIdle"   , "%"     , -30  , 30   , sampleRateHigh, 2, 0},
        {"LTFTCruise" , "%"     , -30  , 30   , sampleRateHigh, 2, 0},
        {"MIVInAct"   , "deg"   , -2.5 , 37.5 , sampleRateHigh, 2, 0},
        {"MIVExAct"   , "deg"   , -37.5, 2.5  , sampleRateHigh, 2, 0},
        {"MAFVolt"    , "V"     , 0    , 5    , sampleRateHigh, 2, 0},
        {"LoadTiming" , "Load"  , 0    , 500  , sampleRateHigh, 1, 0},
        {"LoadMAP"    , "Load"  , 0    , 500  , sampleRateHigh, 1, 0},
        {"LoadIMAP"   , "Load"  , 0    , 500  , sampleRateHigh, 1, 0},
        {"LoadMAF"    , "Load"  , 0    , 500  , sampleRateHigh, 1, 0},
        {"LoadChosen" , "Load"  , 0    , 500  , sampleRateHigh, 1, 0},
        {"Ethanol"    , "%"     , 0    , 100  , sampleRateLow , 1, 0},
//        {"AFR"        , "afr"   , 7    , 23   , sampleRateHigh, 2, 0},
    };
    for(int i = 0; i < CHANNEL_COUNT; i++)
    {    chan_id[i] = create_virtual_channel(cc[i]);
        if (INVALID_VIRTUAL_CHANNEL == chan_id[i])
                return luaL_error(L, "Unable to create channel. "
                                     "Maximum channels reached.");
    }
    chLoad        = chan_id[0];
    chRPM         = chan_id[1];
    chBoost       = chan_id[2];
    chIPW         = chan_id[3];
    chTiming      = chan_id[4];
    chKnockSum    = chan_id[5];
    chTPS         = chan_id[6];
    chAPP         = chan_id[7];
    chMAP         = chan_id[8];
    chBaro        = chan_id[9];
    chWGDC        = chan_id[10];
    chWGDCCorr    = chan_id[11];
    chECT         = chan_id[12];
    chIAT         = chan_id[13];
    chMAT         = chan_id[14];
    chSpeed       = chan_id[15];
    chKnockBase   = chan_id[16];
    chKnockFilter = chan_id[17];
    chSTFT        = chan_id[18];
    chLTFT        = chan_id[19];
    chLTFTIdle    = chan_id[20];
    chLTFTCruise  = chan_id[21];
    chMIVInAct    = chan_id[22];
    chMIVExAct    = chan_id[23];
    chMAFVolt     = chan_id[24];
    chLoadTiming  = chan_id[25];
    chLoadMAP     = chan_id[26];
    chLoadIMAP    = chan_id[27];
    chLoadMAF     = chan_id[28];
    chLoadChosen  = chan_id[29];
    chEthanol     = chan_id[30];
    lua_newtable(L);
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        lua_pushstring(L, cc[i].label);
        lua_pushnumber(L, chan_id[i]);
        lua_rawset(L, -3);
    }
    return 1;
}

void formatCANRequest(unsigned char* data, int pid, int reqLength)
{
    if(pid > 255) { //MODE 23
        data[0] = 5;
        data[1] = 0x23;
        data[2] = pid >> 16;
        data[3] = (pid >> 8) & 0xFF;
        data[4] = pid & 0xFF;
        data[5] = reqLength;
        data[6] = 255;
        data[7] = 255;
    } else { //MODE 21
        data[0] = 2;
        data[1] = 0x21;
        data[2] = pid;
        data[3] = 255;
        data[4] = 255;
        data[5] = 255;
        data[6] = 255;
        data[7] = 255;
    }
}

void pr_debug_CanData(const char *header, CAN_msg *msg)
{
    if (DEBUG_LEVEL) {
        char buf[512];
        sprintf(buf, "%s: address=%d, size=%d, data=%d,%d,%d,%d,%d,%d,%d,%d\r\n", header, msg->addressValue, msg->dataLength,
            msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
        pr_debug(buf);
    }
}

unsigned char req_continue[CAN_MSG_SIZE] = {48,8,0,255,255,255,255,255};
int requestAndReadCAN_impl(unsigned char* data, int* reqLength, lua_State *L) //channel,address,isExtendedAddress,pid,reqLength,timeout
{
    size_t args = lua_gettop(L);
    if (args < 5 || args > 6)
        return incorrect_arguments(L);
    CAN_msg tx_msg;
    uint8_t channel = (uint8_t)lua_tointeger(L, 1);
    tx_msg.addressValue = (unsigned int)lua_tointeger(L, 2);
    tx_msg.isExtendedAddress = lua_tointeger(L, 3);
    int pid = lua_tointeger(L, 4);
    *reqLength = lua_tointeger(L, 5);
    size_t timeout = args >= 6 ? lua_tointeger(L, 6) : DEFAULT_CAN_TIMEOUT;
    formatCANRequest(tx_msg.data, pid, *reqLength);
    tx_msg.dataLength = 8;
    pr_debug_CanData("tx can data", &tx_msg);
    int rc = CAN_tx_msg(channel, &tx_msg, timeout);
    pr_debug_int_msg("CAN_tx_msg rc=", rc);
    if(!rc)
        return 0;

    CAN_msg rx_msg;
    rc = CAN_rx_msg(channel, &rx_msg, timeout);
    pr_debug_int_msg("CAN_rx_msg rc=", rc);
    if(!rc)
        return 0;
    pr_debug_CanData("rx can data", &rx_msg);
    int offset = 2;
    int nbMessages = 0;
    if (rx_msg.data[0] == 0x10) { //Multimessaging
        offset++;
        nbMessages = (rx_msg.data[1] + 1) / 7;
    }
    if (pid < 256) { //Mode23 has more data
        offset++;
        *reqLength = rx_msg.dataLength;
    }
    // else
    //     int rx_pid = msg.data[2];
    int pos = 0;
    for(int i = 0; i + offset < 8; i++, pos++)
      data[pos]=rx_msg.data[i+offset];

    if (nbMessages) {
        memcpy(tx_msg.data, req_continue, CAN_MSG_SIZE);
        pr_debug_CanData("tx can data continue", &tx_msg);
        int rc = CAN_tx_msg(channel, &tx_msg, timeout);
        pr_debug_int_msg("CAN_tx_msg_continue rc=", rc);
        if(!rc)
            return 0;
        for(int m=0; m<nbMessages; m++) {
            rc = CAN_rx_msg(channel, &rx_msg, timeout);
            pr_debug_int_msg("CAN_rx_msg_continue rc=", rc);
            if(!rc)
                return 0;
            pr_debug_CanData("rx can data continue", &rx_msg);
            offset=1;
            for(int i = 0; (i + offset < 8)&&(pos<*reqLength); i++, pos++)
              data[pos]=rx_msg.data[i+offset];
        }
    }
    return 1;
}

int Lua_requestAndReadCAN(lua_State *L) //channel,address,isExtendedAddress,pid,reqLength,timeout
{
    unsigned char data[CAN_MSG_SIZE * 4];
    int reqLength;
    if(!requestAndReadCAN_impl(data, &reqLength, L))
        return 0;
    lua_newtable(L);
    for (int i = 1; i <= reqLength; i++) {
        lua_pushnumber(L, i);
        lua_pushnumber(L, data[i - 1]);
        lua_rawset(L, -3);
    }
    return 1;
}

float get16bitsMSB(unsigned char l, unsigned char r) {
    return (float)(((int)l)*256 + r);
}

float to_float(unsigned char c) {
    return (float)c;
}

static int chRearO2 = -1;
int Lua_set_RearO2_ChannelId(lua_State *L) {
    if (lua_gettop(L) != 1)
        return incorrect_arguments(L);
    chRearO2 = lua_tointeger(L, 1);
    return 1;
}

int Lua_requestAndReadCANAndDecodeTephraV3(lua_State *L) //channel,address,isExtendedAddress,pid,reqLength,timeout
{
    unsigned char data[CAN_MSG_SIZE * 4];
    int reqLength;
    if(!requestAndReadCAN_impl(data, &reqLength, L))
        return 0;
    set_virtual_channel_value(chLoad,get16bitsMSB(data[0], data[1])*10/32);
    set_virtual_channel_value(chRPM,get16bitsMSB(data[2], data[3])*1000/256);
    set_virtual_channel_value(chBoost,(get16bitsMSB(data[4], data[5])/4)*0.19347-14.5);
    set_virtual_channel_value(chMIVInAct, get16bitsMSB(data[6], data[7])*(-10/512)+80);
    set_virtual_channel_value(chMIVExAct, get16bitsMSB(data[8], data[9])*(-10/512)+80);
    set_virtual_channel_value(chIPW,get16bitsMSB(data[10], data[11])/1000);
    if(chRearO2 != -1)
        set_virtual_channel_value(chRearO2,data[12]);
    set_virtual_channel_value(chSpeed,data[13]);
    set_virtual_channel_value(chTiming,data[14]-20);
    set_virtual_channel_value(chKnockSum,data[15]);
    set_virtual_channel_value(chTPS,to_float(data[16])*100.0/255.0);
    set_virtual_channel_value(chWGDC,to_float(data[17])/2.0);
    set_virtual_channel_value(chECT,data[18]-40);
    set_virtual_channel_value(chMAT,data[19]-40);
    set_virtual_channel_value(chIAT,data[20]-40);
    set_virtual_channel_value(chSTFT,to_float(data[21])*0.1953125-25);
    set_virtual_channel_value(chLTFT,to_float(data[22])*0.1953125-25);
    set_virtual_channel_value(chMAFVolt,to_float(data[23])*5/255);
    set_virtual_channel_value(chLoadMAP,get16bitsMSB(data[24], data[25])*100/16384);
    set_virtual_channel_value(chLoadMAF,get16bitsMSB(data[26], data[27])*100/16384);
    set_virtual_channel_value(chKnockBase,data[28]);
    set_virtual_channel_value(chKnockFilter,data[29]);
    return 1;
}

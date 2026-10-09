// Desktop hardware model for the LinnStrument firmware (the "HAL"). It stands in for the SAM3X8E and
// the instrument's electronics below the Arduino API: the clock, the touch sensor (reached over SPI),
// the LEDs (SPI), the UART that carries MIDI, the digital pins and the flash bank that holds settings.
// The firmware itself is compiled unchanged, apart from the rewrites in tools/fwlib/longfix.py and
// tools/fwlib/memfix.py.
//
// Everything is deterministic: the same firmware, inputs and harness always give the same output.
// This header is the model's API; tools/README.md explains how to build and run it.
#ifndef LINNSTRUMENT_DESKTOP_HAL_H
#define LINNSTRUMENT_DESKTOP_HAL_H

#include <stddef.h>
#include <stdint.h>

namespace hal {

// ---- Clock
// Virtual time in nanoseconds since reset. micros() and millis() are derived from it and wrap at 32
// bits like the instrument's. It only moves when the firmware does something that takes time on the
// instrument: an ADC conversion (Timing::adcReadNs), one pass of a busy-wait loop
// (Timing::busyWaitPassNs), the core's delay() and delayMicroseconds(), or a UART write that has to
// wait for room. Plain computation takes no time, so adding code that doesn't read sensors or wait
// can't shift any timestamp.
uint64_t nowNs();
void advanceNs(uint64_t ns);

// The clock reads inside the firmware's busy-waits (delayUsec() and serialWaitForMaximumTwoSeconds(),
// see BUSY_WAITS in tools/fwlib/longfix.py): each advances the clock by one pass, then reads it.
uint32_t busyWaitMicros();
uint32_t busyWaitMillis();

struct Timing {
  uint32_t adcReadNs;       // charged after each ADC conversion (default: 32500, so that an idle
                            // surface scan of 201 cells takes 6.56 ms, as measured on a LinnStrument 200)
  uint32_t busyWaitPassNs;  // one pass of a busy-wait loop; in delayUsec(), performContinuousTasks()
                            // with nothing due plus micros() (default: 3000, an estimate of about
                            // 250 cycles at 84 MHz)
  uint32_t stallReads;      // clock reads with no time passing before the clock creeps forward by
                            // 1 us per read, so a busy-wait with no delay in it still ends (default 100000)
};
extern Timing timing;

// ---- Touch sensor
// The firmware selects a cell and an axis over SPI (selectSensorCell), then reads a 12-bit ADC value
// (spiAnalogRead). The sensor model supplies that ADC value. For Z the firmware inverts it
// (raw = 4095 - adc) and then applies the sensitivity and the column bias; X and Y are used as read.
enum SensorAxis { SENSOR_X = 0, SENSOR_Y = 1, SENSOR_Z = 2 };
typedef uint16_t (*SensorModel)(uint8_t col, uint8_t row, SensorAxis axis, uint64_t tNs, void* user);
void setSensorModel(SensorModel model, void* user);
uint16_t untouchedSurface(uint8_t col, uint8_t row, SensorAxis axis, uint64_t tNs, void* user);
// Counts the firmware's cell and axis selections. The ADC reads that follow one selection share its
// number, so a sensor model can tell a second read within one selection (readZ()'s settling read)
// from a new read of the same cell.
uint64_t sensorSelection();

// ---- UART
// One UART carries MIDI (DIN at 31250 baud, USB at 115200 through the USB-MIDI chip) and serial mode.
// The hook sees every byte the firmware writes: when it was written, and when it starts on the wire.
typedef void (*UartTxHook)(uint64_t writtenNs, uint64_t wireStartNs, uint8_t value, void* user);
void setUartTxHook(UartTxHook hook, void* user);
void uartInject(const uint8_t* data, size_t n);   // bytes for the firmware to receive (MIDI in, serial)
uint32_t uartBaud();                               // 0 until the firmware calls Serial.begin()

// ---- LEDs
// The firmware refreshes one LED column at a time over SPI (refreshLedColumn()): the column's address,
// then its blue, green and red bytes. The hook sees each byte as it's sent, so that two builds can be
// compared on what the LEDs show, and when.
typedef void (*LedSpiHook)(uint64_t tNs, uint8_t value, void* user);
void setLedSpiHook(LedSpiHook hook, void* user);

// ---- Digital pins
// Inputs read HIGH by default: pin 38 HIGH means a LinnStrument 200, and the footswitch inputs (33
// left, 34 right) idle HIGH through their pull-ups. The firmware takes the footswitch level it sees
// at boot as "released", so a normally-open pedal is pressed by setting its pin LOW after setup().
void setInputPin(uint32_t pin, int level);
int outputPin(uint32_t pin);

// ---- Flash: the second flash bank (0xC0000, 256 KB), which DueFlashStorage addresses from 0.
// Erased (all 0xFF) at reset, like the instrument after a firmware update.
uint8_t* flash();
size_t flashSize();

// ---- Counters, for run summaries
struct Counters {
  uint64_t adcReads;        // ADC conversions (sensor reads)
  uint64_t ledSpiBytes;     // bytes sent to the LED drivers
  uint64_t uartTxBytes;     // bytes written to the UART
  uint64_t uartWaitNs;      // time spent waiting for room in the UART buffer
  uint64_t creepNs;         // time added by the stall creep (should stay 0 while playing)
};
const Counters& counters();

}  // namespace hal

#endif

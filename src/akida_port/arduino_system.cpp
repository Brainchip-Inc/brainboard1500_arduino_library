#include "infra/system.h"

#include <Arduino.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>

extern "C" {

void msleep(uint32_t duration) { delay(duration); }

int64_t time_ms(void) { return static_cast<int64_t>(millis()); }

void kick_watchdog(void) {
  yield();
}

void panic(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  va_list args_copy;
  va_copy(args_copy, args);
  std::vsnprintf(buffer, sizeof(buffer), format, args_copy);
  va_end(args_copy);
#ifdef ARDUINO
  if (Serial) {
    Serial.print("[AKIDA][PANIC] ");
    Serial.println(buffer);
    Serial.flush();
  }
#else
  std::printf("[AKIDA][PANIC] %s\r\n", buffer);
  std::printf("\r\n");
  std::fflush(stdout);
#endif
  va_end(args);

  for (;;) {
    delay(1000);
  }
}

}  // extern "C"

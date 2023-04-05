#ifndef LOGGER_H_
#define LOGGER_H_

#define USE_LOG

#ifdef USE_LOG
  void LOG_PRIV(const char* msg);
  #define LOG(msg) LOG_PRIV(msg)
#else
  #define LOG(msg)
#endif  // USE_LOG

#endif  // LOGGER_H_

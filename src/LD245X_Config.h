#ifndef __LD245X_CONFIG_H__
#define __LD245X_CONFIG_H__

// ========== Circular Buffer Configuration ==========
#ifndef LD245X_USE_CIRCULAR_BUFFER
  #define LD245X_USE_CIRCULAR_BUFFER 1
#endif

#ifndef LD245X_RX_BUFFER_SIZE
  #define LD245X_RX_BUFFER_SIZE 2048
#endif

#ifndef LD2420_RX_BUFFER_SIZE
  #define LD2420_RX_BUFFER_SIZE 1024
#endif

// ========== Math Optimization ==========
#ifndef LD245X_USE_FAST_MATH
  #define LD245X_USE_FAST_MATH 1
#endif

#ifndef LD245X_USE_INTEGER_MATH
  #define LD245X_USE_INTEGER_MATH 0  // Set to 1 for platforms without FPU
#endif

// ========== Performance Monitoring ==========
#ifndef LD245X_ENABLE_PERFORMANCE_STATS
  #define LD245X_ENABLE_PERFORMANCE_STATS 1
#endif

// ========== Target History ==========
#ifndef LD245X_ENABLE_TARGET_HISTORY
  #define LD245X_ENABLE_TARGET_HISTORY 0
#endif

#ifndef LD245X_TARGET_HISTORY_SIZE
  #define LD245X_TARGET_HISTORY_SIZE 10
#endif

// ========== Boyer-Moore Pattern Matching ==========
#ifndef LD245X_USE_BMH_SEARCH
  #define LD245X_USE_BMH_SEARCH 0  // Experimental - set to 1 to enable
#endif

#endif // __LD245X_CONFIG_H__

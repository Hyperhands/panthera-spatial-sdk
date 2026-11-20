#pragma once
#include <stdbool.h>
struct gpio_dt_spec { int unused; };
#define DT_ALIAS(x) 0
#define GPIO_DT_SPEC_GET(...) {0}
#define GPIO_OUTPUT_INACTIVE 0
static inline bool gpio_is_ready_dt(const struct gpio_dt_spec *p) { (void)p;return true; }
static inline int gpio_pin_configure_dt(const struct gpio_dt_spec *p,int flags) { (void)p;(void)flags;return 0; }
static inline int gpio_pin_set_dt(const struct gpio_dt_spec *p,int v) { (void)p;(void)v;return 0; }
#define GPIO_INPUT 1
#ifdef TEST_HOME_BUTTON
extern int test_home_button;
static inline int gpio_pin_get_dt(const struct gpio_dt_spec *p) { (void)p;return test_home_button; }
#endif

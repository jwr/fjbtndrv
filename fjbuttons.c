#include <linux/version.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/types.h>

MODULE_AUTHOR("Jan Rychter");
MODULE_DESCRIPTION("FMV Stylistic tablet buttons driver");
MODULE_LICENSE("Dual BSD/GPL");


int init_module(void) {
  return 0;
}

void cleanup_module(void) {
  return;
}

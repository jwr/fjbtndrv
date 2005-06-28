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
#include <linux/ioport.h>
#include <asm/io.h>

MODULE_AUTHOR("(C) 2005 Jan Rychter <jan@rychter.com>");
MODULE_DESCRIPTION("FMV Stylistic tablet buttons driver");
MODULE_LICENSE("Dual BSD/GPL");

#define FJBUTTONS_BASE 			0xfd70
#define FJBUTTONS_ADDRESS_PORT	 	0xfd70
#define FJBUTTONS_RESET_PORT 		0xfd72
#define FJBUTTONS_DATA_PORT		0xfd74
#define FJBUTTONS_STATUS_PORT		0xfd76

/* rotation state: FF-unknown, FD-vertical, FC-horizontal */
#define FJBUTTONS_ROTATION_REGISTER	0xdd

#define FJBUTTONS_DOCK_BASE	0xfd64
#define FJBUTTONS_DOCK_WRITE	0xfd64
#define FJBUTTONS_DOCK_READ	0xfd65

#define FJBUTTONS_DOCK_READ_STATE 0x52  /* command for reading dock state */

int fjbuttons_docked = 0;
int fjbuttons_rotation = 0;
int fjbuttons_irq = 0;
void *fjbuttons_port1 = 0;
void *fjbuttons_port2 = 0;

static char *fjbuttons_driver_name = "fjbtndrv";


int fjbuttons_busywait(void) {
  /* busy wait until 0xfd76 & 0x02 is 0 */
  int timeout_counter = 255;
  do {
    if(!(inb(FJBUTTONS_STATUS_PORT) & 2))
      break;
    timeout_counter--;
  } while(timeout_counter);
  if(!timeout_counter) {
    printk(KERN_INFO "fjbtndrv: timeout waiting for controller to be ready, resetting...\n");
  }
  return timeout_counter;
}


void fjbuttons_reset_until_ready(void) {
  /* read from 0xfd72 */
  /* read from 0xfd76 */
  /* if bit1 set, loop back */
  int timeout_counter = 1024;
  int val;
  do {
    inb(FJBUTTONS_RESET_PORT);
    val = inb(FJBUTTONS_STATUS_PORT) & 1;
    if(!--timeout_counter) {
      printk(KERN_INFO "fjbtndrv: timeout flushing, this should not happen.\n");
      break;
    }
  } while(val);
}



u8 fjbuttons_read_register(u8 reg) {
  outb(reg, FJBUTTONS_ADDRESS_PORT);
  return inb(FJBUTTONS_DATA_PORT);
}


void fjbuttons_write_register(u8 reg, u8 value) {
  outb(reg, FJBUTTONS_ADDRESS_PORT);
  outb(value, FJBUTTONS_DATA_PORT);
}


unsigned int fjbuttons_read_dock_state(void) {
  unsigned int dock_state;
  outb(0x52, FJBUTTONS_DOCK_WRITE);
  dock_state = inb(FJBUTTONS_DOCK_READ) & 0x10;
  if(dock_state) {
    printk(KERN_INFO "fjbtndrv: docked.\n");
  } else {
    printk(KERN_INFO "fjbtndrv: not docked.\n");
  }
  return dock_state;
}


void fjbuttons_reset(void) {
  inb(FJBUTTONS_RESET_PORT);
  /* busy wait until 0xfd76 & 0x02 is 0 */
  //  if(!fjbuttons_busywait()) {
  fjbuttons_busywait();
  if(1) {
    fjbuttons_write_register(0xe8, 0x02);
    fjbuttons_write_register(0xe9, 0x0c);
    fjbuttons_write_register(0xea, 0x05);
    outb(0, FJBUTTONS_STATUS_PORT);
  }

  fjbuttons_reset_until_ready();
  //  fjbuttons_read_rotation_state();  rotation = fjbuttons_read_register(0xdd);
  //fjbuttons_read_dock_state();
}


void fjbuttons_cleanup(void) {
  release_region(FJBUTTONS_BASE, 8);
  release_region(FJBUTTONS_DOCK_BASE, 2);
  free_irq(5, NULL);
}


irqreturn_t fjbuttons_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
  unsigned int irqack = inb(FJBUTTONS_STATUS_PORT);
  u16 keycode;
  u16 state;

  if(!(irqack & 1)) {
    printk(KERN_INFO "fjbtndrv: Interrupt received, but it isn't ours.\n");
    return IRQ_HANDLED;
  }

  //state = fjbuttons_read_register(0xdd);
  //printk(KERN_INFO "fjbtndrv: rotation state: %02x\n", state);
  keycode = fjbuttons_read_register(0xdf);
  keycode |= (fjbuttons_read_register(0xde) << 8);
  printk(KERN_INFO "fjbtndrv: keycodes %04x\n", keycode);
  inb(FJBUTTONS_RESET_PORT);
  return IRQ_HANDLED;
}


static int __init fjbuttons_init(void) {
  printk(KERN_INFO "fjbuttons: initializing...\n");
  fjbuttons_port1 = request_region(FJBUTTONS_BASE, 8, fjbuttons_driver_name);
  fjbuttons_port2 = request_region(FJBUTTONS_DOCK_BASE, 2, fjbuttons_driver_name);
  request_irq(5, fjbuttons_irq_handler, SA_SAMPLE_RANDOM, fjbuttons_driver_name, NULL);
  fjbuttons_reset();
  return 0;
}


static void __exit fjbuttons_exit(void) {
  fjbuttons_cleanup();
  return;
}


module_init(fjbuttons_init)
module_exit(fjbuttons_exit)

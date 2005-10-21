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
#include <linux/input.h>

MODULE_AUTHOR("(C) 2005 Jan Rychter <jan@rychter.com>");
MODULE_DESCRIPTION("Fujitsu FMV Stylistic tablet buttons driver");
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
int fjbuttons_pressed_keys = 0;
int fjbuttons_pressed_modifiers = 0;
void *fjbuttons_port1 = 0;
void *fjbuttons_port2 = 0;

static char *fjbuttons_driver_name = "fjbtndrv";

static struct input_dev fjbuttons_dev;

#define FJB_FN		0x400
#define FJB_ALT 	0x800

#define FJB_1		0x001
#define FJB_2		0x002
#define FJB_3		0x004
#define FJB_4		0x008

#define FJB_PGUP	0x100
#define FJB_PGDN	0x200
#define FJB_UP		0x080
#define FJB_DOWN	0x040

#define FJB_MAX		0x1000

static int fjbuttons_mapping[] = {
  /*key, normal, fn, alt, alt+fn */
  FJB_1, KEY_TAB, 0, KEY_F1, 0,
  FJB_2, KEY_BACKSPACE, 0, KEY_F2, 0,
  FJB_3, KEY_SPACE, 0, KEY_F3, 0,
  FJB_4, KEY_ENTER, 0, KEY_F4, 0,

  FJB_PGUP, KEY_PAGEUP,   KEY_KPMINUS, KEY_HOME, 0,
  FJB_PGDN, KEY_PAGEDOWN, KEY_KPPLUS, KEY_END, 0,

  FJB_UP,   KEY_UP,   KEY_VOLUMEUP,   KEY_LEFT, 0,
  FJB_DOWN, KEY_DOWN, KEY_VOLUMEDOWN, KEY_RIGHT, 0,

  FJB_MAX, 0, 0, 0, 0
};
  

static int fjbuttons_keys[] = { KEY_LEFTALT,
				KEY_TAB,
				KEY_SPACE,
				KEY_BACKSPACE,
				
				KEY_ENTER,

				KEY_VOLUMEDOWN,
				KEY_VOLUMEUP,

				KEY_KPMINUS,
				KEY_KPPLUS,

				KEY_PAGEUP,
				KEY_PAGEDOWN,
				KEY_UP, 
				KEY_DOWN, 
				
				KEY_LEFT,
				KEY_RIGHT,
				KEY_HOME,
				KEY_END,

				KEY_F1,
				KEY_F2,
				KEY_F3,
				KEY_F4,

				KEY_LEFTCTRL,

				KEY_MAX };

void fjbuttons_dev_init(void) {
  int i;
  memset(&fjbuttons_dev, 0, sizeof(fjbuttons_dev));  
  fjbuttons_dev.name = fjbuttons_driver_name;
  set_bit(EV_KEY, fjbuttons_dev.evbit);
  set_bit(EV_REP, fjbuttons_dev.evbit);
  for(i=0; fjbuttons_keys[i] != KEY_MAX; i++)
    set_bit(fjbuttons_keys[i], fjbuttons_dev.keybit);
  input_register_device(&fjbuttons_dev);
}

void fjbuttons_dev_uninit(void) {
  input_unregister_device(&fjbuttons_dev);
}


u16 fjbuttons_dev_translate(u16 code) {
  int i = 0;
  int *ptr;
  while(*(ptr = &fjbuttons_mapping[i*5]) != FJB_MAX) {
    if(ptr[0] == code) {
      return ptr[fjbuttons_pressed_modifiers + 1];
    }
    i++;
  }
  return 0;
}


void fjbuttons_dev_handle(u16 code, int updown) {
  int key=fjbuttons_dev_translate(code);
  if(key) {
    input_report_key(&fjbuttons_dev, key, updown);
    input_sync(&fjbuttons_dev);
  }
}

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
  dock_state = inb(FJBUTTONS_DOCK_READ);
  //  printk(KERN_INFO "fjbtndrv: reading dock register: %02x\n", ~dock_state);
  dock_state = !(dock_state & 0x10);
  fjbuttons_docked = dock_state;
  return dock_state;
}


unsigned int fjbuttons_read_rotation_state(void) {
  unsigned int rotation = fjbuttons_read_register(0xdd);
  //  printk(KERN_INFO "fjbtndrv: rotation register: %02x\n", ~rotation);
  rotation &= 1;
  return fjbuttons_rotation = rotation;
}

void fjbuttons_reset(void) {
  inb(FJBUTTONS_RESET_PORT);
  /* busy wait until 0xfd76 & 0x02 is 0 */
  if(!fjbuttons_busywait()) {
    fjbuttons_write_register(0xe8, 0x02);
    fjbuttons_write_register(0xe9, 0x0c);
    fjbuttons_write_register(0xea, 0x05);
    outb(0, FJBUTTONS_STATUS_PORT);
  }

  fjbuttons_reset_until_ready();
  fjbuttons_read_rotation_state();  
  fjbuttons_read_dock_state();
}


void fjbuttons_cleanup(void) {
  free_irq(5, NULL);
  fjbuttons_dev_uninit();
  release_region(FJBUTTONS_BASE, 8);
  release_region(FJBUTTONS_DOCK_BASE, 2);
}


irqreturn_t fjbuttons_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
  unsigned int irqack = inb(FJBUTTONS_STATUS_PORT);
  u16 keycode;
  u16 state;
  u16 modifiers;

  if(!(irqack & 1)) {
    printk(KERN_INFO "fjbtndrv: Interrupt received, but it isn't ours.\n");
    return IRQ_HANDLED;
  }

  state = fjbuttons_read_rotation_state();
  fjbuttons_read_dock_state();
  keycode = fjbuttons_read_register(0xde);
  keycode |= (fjbuttons_read_register(0xdf) << 8);
  keycode = ~keycode;

  modifiers = (keycode & 0xc000) >> 14;
  fjbuttons_pressed_modifiers = modifiers;  /* todo: check what has changed */

  keycode = (keycode & ~0xc000) >> 4;

  if(keycode != fjbuttons_pressed_keys)
    fjbuttons_dev_handle(fjbuttons_pressed_keys, 0);
  fjbuttons_pressed_keys = keycode;
  //  printk(KERN_INFO "fjbtndrv: keycodes %04hx\n", ~keycode);
  fjbuttons_dev_handle(keycode, 1);
  inb(FJBUTTONS_RESET_PORT);
  return IRQ_HANDLED;
}


static int __init fjbuttons_init(void) {
  printk(KERN_INFO "fjbuttons: initializing...\n");
  fjbuttons_port1 = request_region(FJBUTTONS_BASE, 8, fjbuttons_driver_name);
  fjbuttons_port2 = request_region(FJBUTTONS_DOCK_BASE, 2, fjbuttons_driver_name);
  request_irq(5, fjbuttons_irq_handler, SA_SAMPLE_RANDOM, fjbuttons_driver_name, NULL);
  fjbuttons_reset();
  fjbuttons_dev_init();
  return 0;
}


static void __exit fjbuttons_exit(void) {
  fjbuttons_cleanup();
  return;
}



module_init(fjbuttons_init)
module_exit(fjbuttons_exit)

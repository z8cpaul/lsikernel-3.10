/*
 * lsi_power_debug.c
 *
 *  Created on: Jul 9, 2014
 *      Author: z8cpaul
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include<linux/sched.h>
#include <asm/uaccess.h>
#include "lsi_power_management.h"

int len, temp;

char *msg;

int lsi_wfi_proc(struct file *filp, char *buf, size_t count, loff_t *offp) {

	pm_debug_read_pwr_registers();
	return 0;
}

int lsi_l2_proc(struct file *filp, char *buf, size_t count, loff_t *offp) {

	pm_dump_L2_registers();
	return 0;
}

int lsi_dickens_proc(struct file *filp, char *buf, size_t count, loff_t *offp) {

	pm_dump_dickens();
	return 0;
}

int lsi_power_proc(struct file *filp, char *buf, size_t count, loff_t *offp) {

	pm_cpu_powerup(4);
	return 0;
}


struct file_operations wfi_fops = { read: lsi_wfi_proc };
struct file_operations l2_fops = { read: lsi_l2_proc };
struct file_operations dickens_fops = { read: lsi_dickens_proc };
struct file_operations power_fops = { read: lsi_power_proc };


void lsi_create_new_proc_entry(void) {
	proc_create("wfi", 0, NULL, &wfi_fops);
	proc_create("l2", 0, NULL, &l2_fops);
	proc_create("dickens", 0, NULL, &dickens_fops);
	proc_create("power", 0, NULL, &power_fops);
}

int lsi_proc_init(void) {
	lsi_create_new_proc_entry();
	return 0;
}

void lsi_proc_cleanup(void) {
	remove_proc_entry("wfi", NULL);
	remove_proc_entry("l2", NULL);
	remove_proc_entry("dickens", NULL);
}

MODULE_LICENSE("GPL");
module_init(lsi_proc_init);
module_exit(lsi_proc_cleanup);

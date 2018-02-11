/*
 * kernel/cpu_stress_test.c
 *
 * Copyright (C) 2018, Sultanxda <sultanxda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/cpu.h>
#include <linux/kernel.h>

#define INIT_DELAY_MS (15000)

static struct delayed_work __percpu *worker;
static struct workqueue_struct *high_prio_wq;

static void cpu_intensive_func(struct work_struct *work)
{
#ifdef CONFIG_ARM64
	/* Just exchange data between registers and the stack */
	while (1) {
		preempt_disable();
		asm volatile(
		"stp	x29, x30, [sp, -96]!\n\t"
		"cmp	x0, 16\n\t"
		"add	x29, sp, 0\n\t"
		"stp	x19, x20, [sp,16]\n\t"
		"stp	x21, x22, [sp,32]\n\t"
		"stp	x23, x24, [sp,48]\n\t"
		"stp	x25, x26, [sp,64]\n\t"
		"str	x27, [sp,80]\n\t"
		"ldp	x19, x20, [sp,16]\n\t"
		"ldp	x21, x22, [sp,32]\n\t"
		"ldp	x23, x24, [sp,48]\n\t"
		"ldp	x25, x26, [sp,64]\n\t"
		"ldr	x27, [sp,80]\n\t"
		"ldp	x29, x30, [sp], 96"
		);
		preempt_enable();
	}
#else
	while (1);
#endif
}

static int __init cpu_stress_test_init(void)
{
	DEFINE_SPINLOCK(lock);
	unsigned long flags;
	int cpu;

	BUG_ON(!(worker = alloc_percpu(typeof(*worker))));
	BUG_ON(!(high_prio_wq = alloc_workqueue("stress_test", WQ_HIGHPRI, 0)));

	/* Online all possible CPUs */
	spin_lock_irqsave(&lock, flags);
	for_each_possible_cpu(cpu) {
		if (!cpu_online(cpu))
			cpu_up(cpu);
	}
	get_online_cpus();
	spin_unlock_irqrestore(&lock, flags);

	for_each_online_cpu(cpu) {
		struct delayed_work *pcpu = per_cpu_ptr(worker, cpu);

		INIT_DELAYED_WORK(pcpu, cpu_intensive_func);
		queue_delayed_work_on(cpu, high_prio_wq, pcpu,
			msecs_to_jiffies(INIT_DELAY_MS));
	}

	return 0;
}
late_initcall(cpu_stress_test_init);

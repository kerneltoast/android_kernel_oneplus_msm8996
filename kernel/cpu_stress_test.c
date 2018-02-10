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
#include <linux/slab.h>

#define INIT_DELAY_MS (10000)

static struct delayed_work __percpu *worker;

static void cpu_intensive_func(struct work_struct *work)
{
	/*
	 * kfree(NULL) is just a safe function call that will never be optimized
	 * away; kfree() safely rejects NULL arguments.
	 */
	while (1)
		kfree(NULL);
}

static int __init cpu_stress_test_init(void)
{
	int cpu;

	BUG_ON(!(worker = alloc_percpu(typeof(*worker))));

	get_online_cpus();
	for_each_online_cpu(cpu) {
		struct delayed_work *pcpu = per_cpu_ptr(worker, cpu);

		INIT_DELAYED_WORK(pcpu, cpu_intensive_func);
		schedule_delayed_work_on(cpu, pcpu,
				msecs_to_jiffies(INIT_DELAY_MS));
	}

	return 0;
}
late_initcall(cpu_stress_test_init);

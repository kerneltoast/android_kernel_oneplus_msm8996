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

static struct work_struct __percpu *worker;
static struct workqueue_struct *high_prio_wq;

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
		struct work_struct *pcpu = per_cpu_ptr(worker, cpu);

		INIT_WORK(pcpu, cpu_intensive_func);
		queue_work_on(cpu, high_prio_wq, pcpu);
	}

	return 0;
}
late_initcall(cpu_stress_test_init);

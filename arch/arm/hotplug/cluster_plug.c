/*
 * Cluster-plug CPU Hotplug Driver
 * Designed for homogeneous ARM big.LITTLE systems
 *
 * Copyright 2015 Sultan Qasim Khan
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/cpufreq.h>

//#define DEBUG_CLUSTER_PLUG
#undef DEBUG_CLUSTER_PLUG

#define CLUSTER_PLUG_MAJOR_VERSION	1
#define CLUSTER_PLUG_MINOR_VERSION	0

#define DEF_HYSTERESIS			(10)
#define DEF_LOAD_THRESH			(70)
#define DEF_SAMPLING_MS			(200)
#define N_BIG_CPUS			4

static struct delayed_work cluster_plug_work;
static struct workqueue_struct *clusterplug_wq;

static unsigned int cluster_plug_active = 0;
module_param(cluster_plug_active, uint, 0664);

static unsigned int hysteresis = DEF_HYSTERESIS;
module_param(hysteresis, uint, 0664);

static unsigned int load_threshold = DEF_LOAD_THRESH;
module_param(load_threshold, uint, 0664);

static unsigned int sampling_time = DEF_SAMPLING_MS;
module_param(sampling_time, uint, 0664);

static unsigned int cur_hysteresis = DEF_HYSTERESIS;

struct cp_cpu_info {
	u64 prev_cpu_wall;
	u64 prev_cpu_idle;
};

static DEFINE_PER_CPU(struct cp_cpu_info, cp_info);

static bool is_big_cpu(unsigned int cpu)
{
	return cpu < N_BIG_CPUS;
}

static unsigned int calculate_loaded_cpus(void)
{
	unsigned int cpu;
	unsigned int loaded_cpus = 0;
	struct cp_cpu_info *l_cp_info;

	for_each_online_cpu(cpu) {
		u64 cur_wall_time, cur_idle_time;
		unsigned int wall_time, idle_time;
		unsigned int cpu_load;
		l_cp_info = &per_cpu(cp_info, cpu);

		/* last parameter 0 means that IO wait is considered idle */
		cur_idle_time = get_cpu_idle_time(cpu, &cur_wall_time, 0);

		wall_time = (unsigned int)
			(cur_wall_time - l_cp_info->prev_cpu_wall);
		l_cp_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
			(cur_idle_time - l_cp_info->prev_cpu_idle);
		l_cp_info->prev_cpu_idle = cur_idle_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		cpu_load = 100 * (wall_time - idle_time) / wall_time;

		if (cpu_load > load_threshold)
			loaded_cpus += 1;
	}

	return loaded_cpus;
}

static void __ref plug_clusters(bool enable_little)
{
	unsigned int cpu;
	int ret;
	bool powerhal_override = false;

#ifdef DEBUG_CLUSTER_PLUG
	if (enable_little) pr_info("enabling little cluster\n");
	else pr_info("disabling little cluster\n");
#endif

	for_each_present_cpu(cpu) {
		if (is_big_cpu(cpu) || enable_little) {
			if (cpu_online(cpu)) continue;
			if (powerhal_override && is_big_cpu(cpu)) continue;
			ret = cpu_up(cpu);

			/* PowerHAL may force big cores offline */
			if (ret == -EPERM && is_big_cpu(cpu)) {
				enable_little = true;
				powerhal_override = true;
			}
		} else {
			cpu_down(cpu);
		}
	}
}

static void __ref cluster_plug_work_fn(struct work_struct *work)
{
	unsigned int loaded_cpus, online_cpus;

	if (cluster_plug_active) {
		online_cpus = num_online_cpus();
		loaded_cpus = calculate_loaded_cpus();
#ifdef DEBUG_CLUSTER_PLUG
		pr_info("loaded_cpus: %u\n", loaded_cpus);
#endif

		if (loaded_cpus >= N_BIG_CPUS-1) {
			cur_hysteresis = hysteresis;
			if (online_cpus <= N_BIG_CPUS)
				plug_clusters(true);
		} else if (cur_hysteresis > 0) {
			cur_hysteresis -= 1;
		} else {
			plug_clusters(false);
		}
	}

	queue_delayed_work(clusterplug_wq, &cluster_plug_work,
		msecs_to_jiffies(sampling_time));
}

int __init cluster_plug_init(void)
{
	pr_info("cluster_plug: version %d.%d by sultanqasim\n",
		 CLUSTER_PLUG_MAJOR_VERSION,
		 CLUSTER_PLUG_MINOR_VERSION);

	clusterplug_wq = alloc_workqueue("clusterplug",
				WQ_HIGHPRI | WQ_UNBOUND, 1);
	INIT_DELAYED_WORK(&cluster_plug_work, cluster_plug_work_fn);
	queue_delayed_work_on(0, clusterplug_wq, &cluster_plug_work,
		msecs_to_jiffies(10));

	return 0;
}

MODULE_AUTHOR("Sultan Qasim Khan <sultanqasim@gmail.com>");
MODULE_DESCRIPTION("'cluster_plug' - A cluster based hotplug for homogeneous"
        "ARM big.LITTLE systems where the big cluster is preferred."
);
MODULE_LICENSE("GPL");

late_initcall(cluster_plug_init);

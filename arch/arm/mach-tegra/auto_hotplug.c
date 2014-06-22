/* Copyright (c) 2012, Will Tisdale <willtisdale@gmail.com>. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/*
 * Generic auto hotplug driver for ARM SoCs. Targeted at current generation
 * SoCs with dual and quad core applications processors.
 * Automatically hotplugs online and offline CPUs based on system load.
 *
 * Not recommended for use with OMAP4460 due to the potential for lockups
 * whilst hotplugging.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*
 * Enable debug output to dump the average
 * calculations and ring buffer array values
 * WARNING: Enabling this causes a ton of overhead
 *
 * FIXME: Turn it into debugfs stats (somehow)
 * because currently it is a sack of shit.
 */
#define DEBUG 0
#define CPUS_AVAILABLE		num_possible_cpus()
#define SAMPLING_PERIODS 	18	/* SAMPLING_PERIODS * MIN_SAMPLING_RATE is the minimum load history which will be averaged */
#define INDEX_MAX_VALUE		(SAMPLING_PERIODS - 1)
#define MIN_SAMPLING_RATE	msecs_to_jiffies(20)	/* MIN_SAMPLING_RATE is scaled based on num_online_cpus() */

/* Control flags */
unsigned char flags;
#define HOTPLUG_DISABLED	(1 << 0)
#define HOTPLUG_PAUSED		(1 << 1)
#define BOOSTPULSE_ACTIVE	(1 << 2)
#define EARLYSUSPEND_ACTIVE	(1 << 3)

/*
 * Load defines:
 * ENABLE_ALL is a high watermark to rapidly online all CPUs
 *
 * ENABLE is the load which is required to enable 1 extra CPU
 * DISABLE is the load at which a CPU is disabled
 * These two are scaled based on num_online_cpus()
 */

static unsigned int enable_all_load_threshold = 500;
static unsigned int enable_load_threshold = 225;
static unsigned int disable_load_threshold = 80;
static unsigned int min_cpu_online = 1;
static unsigned int max_cpu_online = 4;
static unsigned int debug = 0;

module_param(enable_all_load_threshold, int, 0644);
module_param(enable_load_threshold, int, 0644);
module_param(disable_load_threshold, int, 0644);
module_param(min_cpu_online, int, 0644);
module_param(max_cpu_online, int, 0644);
module_param(debug, uint, 0644);

#define dprintk(msg...)		\
do { 				\
	if (debug)		\
		pr_info(msg);	\
} while (0)

struct delayed_work hotplug_decision_work;
struct delayed_work hotplug_unpause_work;
struct work_struct hotplug_online_all_work;
struct work_struct hotplug_online_single_work;
struct delayed_work hotplug_offline_work;

static unsigned int history[SAMPLING_PERIODS];
static unsigned int index;

static void hotplug_decision_work_fn(struct work_struct *work)
{
	unsigned int running, disable_load, sampling_rate, enable_load, avg_running = 0;
	unsigned int online_cpus, available_cpus, i, j;

	online_cpus = num_online_cpus();
	available_cpus = max_cpu_online;
	disable_load = disable_load_threshold * online_cpus; 
	enable_load = enable_load_threshold * online_cpus;
	/*
	 * Multiply nr_running() by 100 so we don't have to
	 * use fp division to get the average.
	 */
	running = nr_running() * 100;
	history[index] = running;

	dprintk("online_cpus is: %d\n", online_cpus);
	dprintk("enable_load is: %d\n", enable_load);
	dprintk("disable_load is: %d\n", disable_load);
	dprintk("index is: %d\n", index);
	dprintk("running is: %d\n", running);


	/*
	 * Use a circular buffer to calculate the average load
	 * over the sampling periods.
	 * This will absorb load spikes of short duration where
	 * we don't want additional cores to be onlined because
	 * the cpufreq driver should take care of those load spikes.
	 */
	for (i = 0, j = index; i < SAMPLING_PERIODS; i++, j--) {
		avg_running += history[j];
		if (unlikely(j == 0))
			j = INDEX_MAX_VALUE;
	}

	/*
	 * If we are at the end of the buffer, return to the beginning.
	 */
	if (unlikely(index++ == INDEX_MAX_VALUE))
		index = 0;

	avg_running = avg_running / SAMPLING_PERIODS;
	dprintk("average_running is: %d\n", avg_running);


	if (likely(!(flags & HOTPLUG_DISABLED))) {
		if (unlikely((avg_running >= enable_all_load_threshold) && (online_cpus < available_cpus))) {
			pr_info("auto_hotplug: Onlining all CPUs, avg running: %d\n", avg_running);
			/*
			 * Flush any delayed offlining work from the workqueue.
			 * No point in having expensive unnecessary hotplug transitions.
			 * We still online after flushing, because load is high enough to
			 * warrant it.
			 * We set the paused flag so the sampling can continue but no more
			 * hotplug events will occur.
			 */
			flags |= HOTPLUG_PAUSED;
			if (delayed_work_pending(&hotplug_offline_work))
				cancel_delayed_work(&hotplug_offline_work);
			schedule_work(&hotplug_online_all_work);
			return;
		} else if (flags & HOTPLUG_PAUSED) {
			schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
			return;
		} else if ((avg_running >= enable_load) && (online_cpus < available_cpus)) {
			pr_info("auto_hotplug: Onlining single CPU, avg running: %d\n", avg_running);
			if (delayed_work_pending(&hotplug_offline_work))
				cancel_delayed_work(&hotplug_offline_work);
			schedule_work(&hotplug_online_single_work);
			return;
		} else if (avg_running <= disable_load) {
			/* Only queue a cpu_down() if there isn't one already pending */
			if (!(delayed_work_pending(&hotplug_offline_work))) {
				pr_info("auto_hotplug: Offlining CPU, avg running: %d\n", avg_running);
				schedule_delayed_work_on(0, &hotplug_offline_work, HZ);
			}
		}
	}

	/*
	 * Reduce the sampling rate dynamically based on online cpus.
	 */
	sampling_rate = MIN_SAMPLING_RATE * (online_cpus * online_cpus);
#if DEBUG
	dprintk("sampling_rate is: %d\n", jiffies_to_msecs(sampling_rate));
#endif
	schedule_delayed_work_on(0, &hotplug_decision_work, sampling_rate);

}

static void __cpuinit hotplug_online_all_work_fn(struct work_struct *work)
{
	int cpu;
	for_each_possible_cpu(cpu) {
		if (likely(!cpu_online(cpu))) {
			cpu_up(cpu);
			dprintk("auto_hotplug: CPU%d up.\n", cpu);
		}
	}
	/*
	 * Pause for 1 second before even considering offlining a CPU
	 */
	schedule_delayed_work(&hotplug_unpause_work, HZ * 1);
	schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
}

static void __cpuinit hotplug_online_single_work_fn(struct work_struct *work)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		if (cpu) {
			if (!cpu_online(cpu)) {
				cpu_up(cpu);
				dprintk("auto_hotplug: CPU%d up.\n", cpu);
				break;
			}
		}
	}
	schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
}

static void hotplug_offline_work_fn(struct work_struct *work)
{
	int cpu;

	for_each_online_cpu(cpu) {
		if (cpu && num_online_cpus() > min_cpu_online) {
			cpu_down(num_online_cpus() - 1);
			dprintk("auto_hotplug: CPU%d down.\n", cpu);
			break;
		}
	}
	schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
}

static void hotplug_unpause_work_fn(struct work_struct *work)
{
	dprintk("auto_hotplug: Clearing pause flag\n");
	flags &= ~HOTPLUG_PAUSED;
}

void hotplug_disable(bool flag)
{
	if (flags & HOTPLUG_DISABLED && !flag) {
		flags &= ~HOTPLUG_DISABLED;
		flags &= ~HOTPLUG_PAUSED;
		dprintk("auto_hotplug: Clearing disable flag\n");
		schedule_delayed_work_on(0, &hotplug_decision_work, 0);
	} else if (flag && (!(flags & HOTPLUG_DISABLED))) {
		flags |= HOTPLUG_DISABLED;
		dprintk("auto_hotplug: Setting disable flag\n");
		cancel_delayed_work_sync(&hotplug_offline_work);
		cancel_delayed_work_sync(&hotplug_decision_work);
		cancel_delayed_work_sync(&hotplug_unpause_work);
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void auto_hotplug_early_suspend(struct early_suspend *handler)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		if (cpu)
			cpu_down(cpu);
	dprintk("auto_hotplug: Offlining CPUs for early suspend\n");
	}
	flags |= EARLYSUSPEND_ACTIVE;

	/* Cancel all scheduled delayed work to avoid races */
	cancel_delayed_work_sync(&hotplug_offline_work);
	cancel_delayed_work_sync(&hotplug_decision_work);
}

static void auto_hotplug_late_resume(struct early_suspend *handler)
{
	dprintk("auto_hotplug: late resume handler\n");
	flags &= ~EARLYSUSPEND_ACTIVE;

	schedule_delayed_work_on(0, &hotplug_decision_work, HZ);
}

static struct early_suspend auto_hotplug_suspend = {
	.suspend = auto_hotplug_early_suspend,
	.resume = auto_hotplug_late_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

int __init auto_hotplug_init(void)
{
	pr_info("auto_hotplug: v0.220 by _thalamus\n");
	dprintk("auto_hotplug: %d CPUs detected\n", CPUS_AVAILABLE);

	INIT_DELAYED_WORK(&hotplug_decision_work, hotplug_decision_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_unpause_work, hotplug_unpause_work_fn);
	INIT_WORK(&hotplug_online_all_work, hotplug_online_all_work_fn);
	INIT_WORK(&hotplug_online_single_work, hotplug_online_single_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_offline_work, hotplug_offline_work_fn);

	/*
	 * Give the system time to boot before fiddling with hotplugging.
	 */
	flags |= HOTPLUG_PAUSED;
	schedule_delayed_work_on(0, &hotplug_decision_work, HZ * 10);
	schedule_delayed_work(&hotplug_unpause_work, HZ * 20);

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&auto_hotplug_suspend);
#endif
	return 0;
}
late_initcall(auto_hotplug_init);

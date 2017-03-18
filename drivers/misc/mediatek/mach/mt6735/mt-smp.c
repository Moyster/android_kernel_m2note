#include <linux/init.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <asm/localtimer.h>
#include <asm/fiq_glue.h>
#include <mach/mt_reg_base.h>
#include <mach/smp.h>
#include <mach/sync_write.h>
#include <mach/hotplug.h>
#include <mach/mt_spm_mtcmos.h>
#include <mach/mt_spm_idle.h>
#include <mach/wd_api.h>
#include <mach/mt_secure_api.h>

#define SLAVE1_MAGIC_REG (SRAMROM_BASE+0x38)
#define SLAVE2_MAGIC_REG (SRAMROM_BASE+0x38)
#define SLAVE3_MAGIC_REG (SRAMROM_BASE+0x38)
#define SLAVE4_MAGIC_REG (SRAMROM_BASE+0x3C)
#define SLAVE5_MAGIC_REG (SRAMROM_BASE+0x3C)
#define SLAVE6_MAGIC_REG (SRAMROM_BASE+0x3C)
#define SLAVE7_MAGIC_REG (SRAMROM_BASE+0x3C)

#define SLAVE1_MAGIC_NUM 0x534C4131
#define SLAVE2_MAGIC_NUM 0x4C415332
#define SLAVE3_MAGIC_NUM 0x41534C33
#define SLAVE4_MAGIC_NUM 0x534C4134
#define SLAVE5_MAGIC_NUM 0x4C415335
#define SLAVE6_MAGIC_NUM 0x41534C36
#define SLAVE7_MAGIC_NUM 0x534C4137

#define SLAVE_JUMP_REG  (SRAMROM_BASE+0x34)


#if 0
#define CA15L_TYPEID 0x410FC0D0
#define CA7_TYPEID 0x410FC070
#define CPU_TYPEID_MASK 0xfffffff0
#define read_midr()							\
	({								\
		register unsigned int ret;				\
		__asm__ __volatile__ ("mrc   p15, 0, %0, c0, c0, 0 \n\t" \
				      :"=r"(ret));			\
		ret;							\
	})

//inline int is_cpu_type(int type)
#define is_cpu_type(type)						\
	({								\
		((read_midr() & CPU_TYPEID_MASK) == type) ? 1 : 0;	\
	})
#endif


extern void mt_secondary_startup(void);
//fix build error
//extern void irq_raise_softirq(const struct cpumask *mask, unsigned int irq);
extern void mt_gic_secondary_init(void);


extern unsigned int irq_total_secondary_cpus;
static DEFINE_SPINLOCK(boot_lock);


/*
 * Write pen_release in a way that is guaranteed to be visible to all
 * observers, irrespective of whether they're taking part in coherency
 * or not.  This is necessary for the hotplug code to work reliably.
 */
static void write_pen_release(int val)
{
    pen_release = val;
    smp_wmb();
    __cpuc_flush_dcache_area((void *)&pen_release, sizeof(pen_release));
    outer_clean_range(__pa(&pen_release), __pa(&pen_release + 1));
}

/*
 * 20140512 marc.huang
 * 1. only need to get core count if !defined(CONFIG_OF)
 * 2. only set possible cpumask in mt_smp_init_cpus() if !defined(CONFIG_OF)
 * 3. only set present cpumask in mt_smp_prepare_cpus() if !defined(CONFIG_OF)
 */
#if !defined(CONFIG_OF)
static int _mt_smp_get_core_count(void)
{
    unsigned int cores = 0;

    //asm volatile(
    //"MRC p15, 1, %0, c9, c0, 2\n"
    //: "=r" (cores)
    //:
    //: "cc"
    //);
    //
    //cores = cores >> 24;
    //cores += 1;

    //TODO: use efuse api to get core numbers?
    cores = 8;

    return cores;
}
#endif //#if !defined(CONFIG_OF)

void mt_smp_secondary_init(unsigned int cpu)
{
#if 0
    struct wd_api *wd_api = NULL;

    //fix build error
    get_wd_api(&wd_api);
    if (wd_api)
        wd_api->wd_cpu_hot_plug_on_notify(cpu);
#endif

    pr_debug("Slave cpu init\n");
    HOTPLUG_INFO("platform_secondary_init, cpu: %d\n", cpu);

    mt_gic_secondary_init();

    /*
     * let the primary processor know we're out of the
     * pen, then head off into the C entry point
     */
    write_pen_release(-1);

#if !defined (CONFIG_ARM_PSCI)
    //cannot enable in secure world
    fiq_glue_resume();
#endif //#if !defined (CONFIG_ARM_PSCI)

    /*
     * Synchronise with the boot thread.
     */
    spin_lock(&boot_lock);
    spin_unlock(&boot_lock);
}

int mt_smp_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
    unsigned long timeout;

    pr_crit("Boot slave CPU\n");

    atomic_inc(&hotplug_cpu_count);

    /*
     * Set synchronisation state between this boot processor
     * and the secondary one
     */
    spin_lock(&boot_lock);

    HOTPLUG_INFO("mt_smp_boot_secondary, cpu: %d\n", cpu);
    /*
     * The secondary processor is waiting to be released from
     * the holding pen - release it, then wait for it to flag
     * that it has been released by resetting pen_release.
     *
     * Note that "pen_release" is the hardware CPU ID, whereas
     * "cpu" is Linux's internal ID.
     */
    /*
     * This is really belt and braces; we hold unintended secondary
     * CPUs in the holding pen until we're ready for them.  However,
     * since we haven't sent them a soft interrupt, they shouldn't
     * be there.
     */
    write_pen_release(cpu);

    mt_smp_set_boot_addr(virt_to_phys(mt_secondary_startup), cpu);
    switch(cpu)
    {
        case 1:
        #ifdef CONFIG_MTK_FPGA
            mt_reg_sync_writel(SLAVE1_MAGIC_NUM, SLAVE1_MAGIC_REG);
            HOTPLUG_INFO("SLAVE1_MAGIC_NUM:%x\n", SLAVE1_MAGIC_NUM);
        #endif
            spm_mtcmos_ctrl_cpu1(STA_POWER_ON, 1);
            break;
        case 2:
        #ifdef CONFIG_MTK_FPGA
            mt_reg_sync_writel(SLAVE2_MAGIC_NUM, SLAVE2_MAGIC_REG);
            HOTPLUG_INFO("SLAVE2_MAGIC_NUM:%x\n", SLAVE2_MAGIC_NUM);
        #endif
            spm_mtcmos_ctrl_cpu2(STA_POWER_ON, 1);
            break;
        case 3:
        #ifdef CONFIG_MTK_FPGA
            mt_reg_sync_writel(SLAVE3_MAGIC_NUM, SLAVE3_MAGIC_REG);
            HOTPLUG_INFO("SLAVE3_MAGIC_NUM:%x\n", SLAVE3_MAGIC_NUM);
        #endif
            spm_mtcmos_ctrl_cpu3(STA_POWER_ON, 1);
            break;
        case 4:
        #ifdef CONFIG_MTK_FPGA
            mt_reg_sync_writel(SLAVE4_MAGIC_NUM, SLAVE4_MAGIC_REG);
            HOTPLUG_INFO("SLAVE4_MAGIC_NUM:%x\n", SLAVE4_MAGIC_NUM);
        #endif
            spm_mtcmos_ctrl_cpu4(STA_POWER_ON, 1);
            break;

        case 5:
            if ((cpu_online(4) == 0) && (cpu_online(6) == 0) && (cpu_online(7) == 0))
            {
                    HOTPLUG_INFO("up CPU%d fail, please up CPU4 first\n", cpu);
                    spin_unlock(&boot_lock);
                    atomic_dec(&hotplug_cpu_count);
                    return -ENOSYS;
            }
        #ifdef CONFIG_MTK_FPGA
            mt_reg_sync_writel(SLAVE5_MAGIC_NUM, SLAVE5_MAGIC_REG);
            HOTPLUG_INFO("SLAVE5_MAGIC_NUM:%x\n", SLAVE5_MAGIC_NUM);
        #endif
            spm_mtcmos_ctrl_cpu5(STA_POWER_ON, 1);
            break;

        case 6:
            if ((cpu_online(4) == 0) && (cpu_online(5) == 0) && (cpu_online(7) == 0))
            {
                    HOTPLUG_INFO("up CPU%d fail, please up CPU4 first\n", cpu);
                    spin_unlock(&boot_lock);
                    atomic_dec(&hotplug_cpu_count);
                    return -ENOSYS;
            }
        #ifdef CONFIG_MTK_FPGA
            mt_reg_sync_writel(SLAVE6_MAGIC_NUM, SLAVE6_MAGIC_REG);
            HOTPLUG_INFO("SLAVE6_MAGIC_NUM:%x\n", SLAVE6_MAGIC_NUM);
        #endif
            spm_mtcmos_ctrl_cpu6(STA_POWER_ON, 1);
            break;

        case 7:
            if ((cpu_online(4) == 0) && (cpu_online(5) == 0) && (cpu_online(6) == 0))
            {
                    HOTPLUG_INFO("up CPU%d fail, please up CPU4 first\n", cpu);
                    spin_unlock(&boot_lock);
                    atomic_dec(&hotplug_cpu_count);
                    return -ENOSYS;
            }
        #ifdef CONFIG_MTK_FPGA
            mt_reg_sync_writel(SLAVE7_MAGIC_NUM, SLAVE7_MAGIC_REG);
            HOTPLUG_INFO("SLAVE7_MAGIC_NUM:%x\n", SLAVE7_MAGIC_NUM);
        #endif
            spm_mtcmos_ctrl_cpu7(STA_POWER_ON, 1);
            break;

        default:
            break;

    }

    //smp_cross_call(cpumask_of(cpu));

    /*
     * Now the secondary core is starting up let it run its
     * calibrations, then wait for it to finish
     */
    spin_unlock(&boot_lock);

    timeout = jiffies + (1 * HZ);
    while (time_before(jiffies, timeout)) {
        smp_rmb();
        if (pen_release == -1)
            break;

        udelay(10);
    }

    if (pen_release == -1)
    {
    #if 0
        //FIXME: update to the right register names
        pr_emerg("SPM_CA7_CPU0_PWR_CON: 0x%08x\n", REG_READ(SPM_CA7_CPU0_PWR_CON));
        pr_emerg("SPM_CA7_CPU1_PWR_CON: 0x%08x\n", REG_READ(SPM_CA7_CPU1_PWR_CON));
        pr_emerg("SPM_CA7_CPU2_PWR_CON: 0x%08x\n", REG_READ(SPM_CA7_CPU2_PWR_CON));
        pr_emerg("SPM_CA7_CPU3_PWR_CON: 0x%08x\n", REG_READ(SPM_CA7_CPU3_PWR_CON));
        pr_emerg("SPM_CA7_DBG_PWR_CON: 0x%08x\n", REG_READ(SPM_CA7_DBG_PWR_CON));
        pr_emerg("SPM_CA7_CPUTOP_PWR_CON: 0x%08x\n", REG_READ(SPM_CA7_CPUTOP_PWR_CON));
        pr_emerg("SPM_CA15_CPU0_PWR_CON: 0x%08x\n", REG_READ(SPM_CA15_CPU0_PWR_CON));
        pr_emerg("SPM_CA15_CPU1_PWR_CON: 0x%08x\n", REG_READ(SPM_CA15_CPU1_PWR_CON));
        pr_emerg("SPM_CA15_CPU2_PWR_CON: 0x%08x\n", REG_READ(SPM_CA15_CPU2_PWR_CON));
        pr_emerg("SPM_CA15_CPU3_PWR_CON: 0x%08x\n", REG_READ(SPM_CA15_CPU3_PWR_CON));
        pr_emerg("SPM_CA15_CPUTOP_PWR_CON: 0x%08x\n", REG_READ(SPM_CA15_CPUTOP_PWR_CON));
    #endif
        return 0;
    }
    else
    {
    #if 0
        //FIXME: how k2 debug monitor
        if (is_cpu_type(CA7_TYPEID))
        {
            //write back stage pc on ca7
            mt_reg_sync_writel(cpu + 8, DBG_MON_CTL);
            pr_emerg("CPU%u, DBG_MON_CTL: 0x%08x, DBG_MON_DATA: 0x%08x\n", cpu, *(volatile u32 *)(DBG_MON_CTL), *(volatile u32 *)(DBG_MON_DATA));
        }
        else
        {
            //decode statge pc on ca15l
            mt_reg_sync_writel(3+ (cpu - 4) * 4, CA15L_MON_SEL);
            pr_emerg("CPU%u, CA15L_MON_SEL: 0x%08x, CA15L_MON: 0x%08x\n", cpu, *(volatile u32 *)(CA15L_MON_SEL), *(volatile u32 *)(CA15L_MON));
        }
    #endif

        on_each_cpu((smp_call_func_t)dump_stack, NULL, 0);
        atomic_dec(&hotplug_cpu_count);
        return -ENOSYS;
    }
}

#if defined (MT_SMP_VIRTUAL_BOOT_ADDR)
/** this routine is to solve BOOT_ADDR concurrence:
 * 1. to provide a boot_addr array for every CPUs as 2nd boot_addr,
 *    for both purpose of dormant and hotplug.
 * 2. a common wakeup routine stocked in BOOT_ADDR,
 *    to jump 2nd boot_addr.
 **/

static unsigned boot_addr_array[8 /* NR_CPUS */];
/** the common wakeup routine:
 * has one assumption:  4 cores per cluster.
 **/
__naked void mt_smp_boot(void)
{
	__asm__ __volatile__ (
		"mrc p15, 0, r0, c0, c0, 5	@ pmidr \n\t"
		"ubfx r1, r0, #8, #4 		@ tid \n\t"
		"and r0, r0, #0xf		@ cid \n\t"
		"add r2, r0, r1, lsl #2		@ idx = tid*4+cid \n\t"
		"adr r3, 1f			@ pa1 \n\t"
		"ldm r3, {r4-r5}		@ va1, va2 \n\t"
		"sub r4, r3, r4			@ off=pa1-va1 \n\t"
		"add r3, r5, r4 		@ pa2=v2+off \n\t"
		"ldr lr, [r3, r2, lsl #2]	@ baddr= array+idx*2 \n\t"
		"bx  lr				@ jump \n\t"
		"1: .long . 		\n\t"
		".long boot_addr_array	\n\t"
		);
}

void mt_smp_set_boot_addr(u32 addr, int cpu)
{
    boot_addr_array[cpu] = addr;
    __cpuc_flush_dcache_area(boot_addr_array, sizeof(boot_addr_array));
}
#endif

void __init mt_smp_init_cpus(void)
{
    /* Enable CA7 snoop function */
#if defined(CONFIG_ARM_PSCI) || defined(CONFIG_MTK_PSCI)
    mcusys_smc_write_phy(virt_to_phys(MP0_AXI_CONFIG), REG_READ(MP0_AXI_CONFIG) & ~ACINACTM);
#else //#if defined(CONFIG_ARM_PSCI) || defined(CONFIG_MTK_PSCI)
    mcusys_smc_write(MP0_AXI_CONFIG, REG_READ(MP0_AXI_CONFIG) & ~ACINACTM);
#endif //#if defined(CONFIG_ARM_PSCI) || defined(CONFIG_MTK_PSCI)

    /* Enable snoop requests and DVM message requests*/
    REG_WRITE(CCI400_SI4_SNOOP_CONTROL, REG_READ(CCI400_SI4_SNOOP_CONTROL) | (SNOOP_REQ | DVM_MSG_REQ));
    while (REG_READ(CCI400_STATUS) & CHANGE_PENDING);

    pr_emerg("@@@### num_possible_cpus(): %u ###@@@\n", num_possible_cpus());
    pr_emerg("@@@### num_present_cpus(): %u ###@@@\n", num_present_cpus());
#if !defined(CONFIG_OF)
    {
    unsigned int i, ncores;
    ncores = _mt_smp_get_core_count();
    if (ncores > NR_CPUS) {
        printk(KERN_WARNING
               "L2CTLR core count (%d) > NR_CPUS (%d)\n", ncores, NR_CPUS);
        printk(KERN_WARNING
               "set nr_cores to NR_CPUS (%d)\n", NR_CPUS);
        ncores = NR_CPUS;
    }

    for (i = 0; i < ncores; i++)
        set_cpu_possible(i, true);
    }
#endif //#if !defined(CONFIG_OF)

    irq_total_secondary_cpus = num_possible_cpus() - 1;

    //fix build error
    //set_smp_cross_call(irq_raise_softirq);

    //XXX: asssume only boot cpu power on and all non-boot cpus power off after preloader stage
    //if (ncores > 4)
    //    spm_mtcmos_ctrl_cpusys1_init_1st_bring_up(STA_POWER_ON);
    //else
    //    spm_mtcmos_ctrl_cpusys1_init_1st_bring_up(STA_POWER_DOWN);
}

void __init mt_smp_prepare_cpus(unsigned int max_cpus)
{
#if !defined (CONFIG_ARM_PSCI)

    /*
     * 20140512 marc.huang
     * 1. only need to get core count if !defined(CONFIG_OF)
     * 2. only set possible cpumask in mt_smp_init_cpus() if !defined(CONFIG_OF)
     * 3. only set present cpumask in mt_smp_prepare_cpus() if !defined(CONFIG_OF)
     */
#if !defined(CONFIG_OF)
    int i;

    for (i = 0; i < max_cpus; i++)
        set_cpu_present(i, true);
#endif //#if !defined(CONFIG_OF)

#ifdef CONFIG_MTK_FPGA
    /* write the address of slave startup into the system-wide flags register */
    mt_reg_sync_writel(virt_to_phys(mt_secondary_startup), SLAVE_JUMP_REG);
#endif

    /* Set all cpus into AArch32 */
    mcusys_smc_write(MP0_MISC_CONFIG3, REG_READ(MP0_MISC_CONFIG3) & 0xFFFF0FFF);
    mcusys_smc_write(MP1_MISC_CONFIG3, REG_READ(MP1_MISC_CONFIG3) & 0xFFFF0FFF);

    /* enable bootrom power down mode */
    REG_WRITE(BOOTROM_SEC_CTRL, REG_READ(BOOTROM_SEC_CTRL) | SW_ROM_PD);

    /* write the address of slave startup into boot address register for bootrom power down mode */
#if defined (MT_SMP_VIRTUAL_BOOT_ADDR)
    mt_reg_sync_writel(virt_to_phys(mt_smp_boot), BOOTROM_BOOT_ADDR);
#else
    mt_reg_sync_writel(virt_to_phys(mt_secondary_startup), BOOTROM_BOOT_ADDR);
#endif

#endif //#if !defined (CONFIG_ARM_PSCI)

    /* initial spm_mtcmos memory map */
    spm_mtcmos_cpu_init();
}

struct smp_operations __initdata mt_smp_ops = {
    .smp_init_cpus          = mt_smp_init_cpus,
    .smp_prepare_cpus       = mt_smp_prepare_cpus,
    .smp_secondary_init     = mt_smp_secondary_init,
    .smp_boot_secondary     = mt_smp_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
    .cpu_kill               = mt_cpu_kill,
    .cpu_die                = mt_cpu_die,
    .cpu_disable            = mt_cpu_disable,
#endif
};

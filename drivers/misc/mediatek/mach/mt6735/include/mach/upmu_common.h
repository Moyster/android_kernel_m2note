#ifndef _MT_PMIC_COMMON_H_
#define _MT_PMIC_COMMON_H_

#include <linux/types.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>

extern unsigned int pmic_read_interface (unsigned int RegNum, unsigned int *val, unsigned int MASK, unsigned int SHIFT);
extern unsigned int pmic_config_interface (unsigned int RegNum, unsigned int val, unsigned int MASK, unsigned int SHIFT);
extern unsigned int pmic_read_interface_nolock (unsigned int RegNum, unsigned int *val, unsigned int MASK, unsigned int SHIFT);
extern unsigned int pmic_config_interface_nolock (unsigned int RegNum, unsigned int val, unsigned int MASK, unsigned int SHIFT);
extern void pmic_lock(void);extern void pmic_unlock(void);
extern void upmu_set_reg_value(unsigned int reg, unsigned int reg_val);

extern unsigned short pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname, unsigned int val);
extern unsigned short pmic_get_register_value(PMU_FLAGS_LIST_ENUM flagname);
extern unsigned short bc11_set_register_value(PMU_FLAGS_LIST_ENUM flagname, unsigned int val);
extern unsigned short bc11_get_register_value(PMU_FLAGS_LIST_ENUM flagname);

extern void pmic_enable_interrupt(unsigned int intNo, unsigned int en, char *str);
extern void pmic_register_interrupt_callback(unsigned int intNo, void (EINT_FUNC_PTR)(void));
extern unsigned short is_battery_remove_pmic(void);

extern signed int PMIC_IMM_GetCurrent(void);
extern unsigned int PMIC_IMM_GetOneChannelValue(mt6328_adc_ch_list_enum dwChannel, int deCount, int trimd);


#endif // _MT_PMIC_COMMON_H_

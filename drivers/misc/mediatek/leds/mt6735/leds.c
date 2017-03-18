/*
 * 
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 * mt65xx leds driver
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/leds-mt65xx.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/delay.h>
/* #include <cust_leds.h> */
/* #include <cust_leds_def.h> */
#include <mach/mt_pwm.h>
#include <mach/mt_boot.h>
/* #include <mach/mt_gpio.h> */
#include <mach/upmu_common_sw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include "leds_sw.h"
#include <linux/aal_api.h>
#include <linux/aee.h>
#include <mach/mt_pmic_wrap.h>

static DEFINE_MUTEX(leds_mutex);
static DEFINE_MUTEX(leds_pmic_mutex);
#ifdef MEIZU_M81
static DEFINE_MUTEX(led_oneshot_mutex);
#endif
/****************************************************************************
 * variables
 ***************************************************************************/
/* struct cust_mt65xx_led* bl_setting_hal = NULL; */
 static unsigned int bl_brightness_hal = 102;
static unsigned int bl_duty_hal = 21;
static unsigned int bl_div_hal = CLK_DIV1;
static unsigned int bl_frequency_hal = 32000;
#ifdef MEIZU_M81
static unsigned int button_brightness_hal = 0;
#endif
//for button led don't do ISINK disable first time
static int button_flag_isink0 = 0;
static int button_flag_isink1 = 0;
static int button_flag_isink2 = 0;
static int button_flag_isink3 = 0;

#ifdef IWLED_SUPPORT
// for backlight to remember the last backlight level status
static int last_level = 1;
#endif
struct wake_lock leds_suspend_lock;

/****************************************************************************
 * DEBUG MACROS
 ***************************************************************************/
static int debug_enable_led_hal = 0;
#define LEDS_DEBUG(format, args...) do{ \
	if(debug_enable_led_hal) \
	{\
		printk(KERN_EMERG format, ##args);\
	}\
}while(0)

/****************************************************************************
 * custom APIs
***************************************************************************/
extern unsigned int brightness_mapping(unsigned int level);
#ifdef MEIZU_M81
extern unsigned int Cust_SetBacklight(unsigned int level);
#endif
/*****************PWM *************************************************/
static int time_array_hal[PWM_DIV_NUM]={256,512,1024,2048,4096,8192,16384,32768};
static unsigned int div_array_hal[PWM_DIV_NUM] = {1,2,4,8,16,32,64,128}; 

static unsigned int backlight_PWM_div_hal = CLK_DIV1;	/* this para come from cust_leds. */

/******************************************************************************
   for DISP backlight High resolution 
******************************************************************************/
#ifdef LED_INCREASE_LED_LEVEL_MTKPATCH
#define MT_LED_INTERNAL_LEVEL_BIT_CNT 10
#endif

/* Use Old Mode of PWM to suppoort 256 backlight level */
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256
extern unsigned int Cust_GetBacklightLevelSupport_byPWM(void);

#ifdef MEIZU_M81
int mt_brightness_set_pmic_level(enum mt65xx_led_pmic pmic_type, u32 level);
#endif
/****************************************************************************
 * func:return global variables
 ***************************************************************************/

void mt_leds_wake_lock_init(void)
{
	wake_lock_init(&leds_suspend_lock, WAKE_LOCK_SUSPEND, "leds wakelock");
}

unsigned int mt_get_bl_brightness(void)
{
	return bl_brightness_hal;
}

unsigned int mt_get_bl_duty(void)
{
	return bl_duty_hal;
}

unsigned int mt_get_bl_div(void)
{
	return bl_div_hal;
}

unsigned int mt_get_bl_frequency(void)
{
	return bl_frequency_hal;
}

unsigned int *mt_get_div_array(void)
{
	return &div_array_hal[0];
}

void mt_set_bl_duty(unsigned int level)
{
	bl_duty_hal = level;
}

void mt_set_bl_div(unsigned int div)
{
	bl_div_hal = div;
}

void mt_set_bl_frequency(unsigned int freq)
{
	 bl_frequency_hal = freq;
}

struct cust_mt65xx_led * mt_get_cust_led_list(void)
{
	return get_cust_led_list();
}


/****************************************************************************
 * internal functions
 ***************************************************************************/
/* #if 0 */
static int brightness_mapto64(int level)
{
        if (level < 30)
                return (level >> 1) + 7;
        else if (level <= 120)
                return (level >> 2) + 14;
        else if (level <= 160)
                return level / 5 + 20;
        else
                return (level >> 3) + 33;
}



static int find_time_index(int time)
{	
	int index = 0;	
	while (index < 8) {
		if(time<time_array_hal[index])			
			return index;		
		else
			index++;
	}	
	return PWM_DIV_NUM-1;
}

int mt_led_set_pwm(int pwm_num, struct nled_setting* led)
{
	struct pwm_spec_config pwm_setting;
	int time_index = 0;
	pwm_setting.pwm_no = pwm_num;
    pwm_setting.mode = PWM_MODE_OLD;
    
    LEDS_DEBUG("[LED]led_set_pwm: mode=%d,pwm_no=%d\n", led->nled_mode, pwm_num);  
	/* We won't choose 32K to be the clock src of old mode because of system performance. */
	/* The setting here will be clock src = 26MHz, CLKSEL = 26M/1625 (i.e. 16K) */
	pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;
    
	switch (led->nled_mode)
	{
		case NLED_OFF : /* Actually, the setting still can not to turn off NLED. We should disable PWM to turn off NLED. */
			pwm_setting.PWM_MODE_OLD_REGS.THRESH = 0;
			pwm_setting.clk_div = CLK_DIV1;
			pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100/2;
			break;
            
		case NLED_ON :
			pwm_setting.PWM_MODE_OLD_REGS.THRESH = 30/2;
			pwm_setting.clk_div = CLK_DIV1;			
			pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 100/2;
			break;
            
		case NLED_BLINK :
			LEDS_DEBUG("[LED]LED blink on time = %d offtime = %d\n",led->blink_on_time,led->blink_off_time);
			time_index = find_time_index(led->blink_on_time + led->blink_off_time);
			LEDS_DEBUG("[LED]LED div is %d\n",time_index);
			pwm_setting.clk_div = time_index;
			pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = (led->blink_on_time + led->blink_off_time) * MIN_FRE_OLD_PWM / div_array_hal[time_index];
			pwm_setting.PWM_MODE_OLD_REGS.THRESH = (led->blink_on_time*100) / (led->blink_on_time + led->blink_off_time);
	}
	
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;
	pwm_set_spec_config(&pwm_setting);

	return 0;
}


/************************ led breath funcion*****************************/
/*************************************************************************
//func is to swtich to breath mode from PWM mode of ISINK
//para: enable: 1 : breath mode; 0: PWM mode;
*************************************************************************/
#if 1
static int led_switch_breath_pmic(enum mt65xx_led_pmic pmic_type, struct nled_setting* led, int enable)
{
	/* int time_index = 0; */
	/* int duty = 0; */
#ifdef MEIZU_M81
	unsigned int tr1,tr2,tf1,tf2,ton,toff,duty;
#endif

	LEDS_DEBUG("[LED]led_blink_pmic: pmic_type=%d\n", pmic_type);  
	
	if((pmic_type != MT65XX_LED_PMIC_NLED_ISINK0 && pmic_type!= MT65XX_LED_PMIC_NLED_ISINK1 && 
		pmic_type!= MT65XX_LED_PMIC_NLED_ISINK2 && pmic_type!= MT65XX_LED_PMIC_NLED_ISINK3) || led->nled_mode != NLED_BLINK) {
		return -1;
	}
#ifdef MEIZU_M81
        /** LED isink0 parameters **/
        //0: 0.123s 1: 0.338s 2: 0.523s 3: 0.707s 4: 0.926s 5: 1.107s 6: 1.291s 7: 1.507s
        //8: 1.691s 9: 1.876s 10: 2.091s 11: 2.276s 12: 2.460s 13: 2.676s 14: 2.860s 15: 3.075s
        tr1 = 1;
        tr2 = 1;
        tf1 = 1;
        tf2 = 1;
        ton = 2;
        // 0: 0.246s  1: 0.677s  2: 1.046s  3: 1.417s  4: 1.845s  5: 2.214s  6: 2.583s  7: 3.014s
        // 8: 3.383s  9: 3.752s  10: 4.183s  11: 4.552s  12: 4.921s  13: 5.351s  14: 5.720s  15: 6.151s
        toff = 3;
        // (N+1)/32
        duty = 7;
        /** LED isink0 parameters **/
#endif
  if(1 == enable) {			
	switch(pmic_type){
	#ifdef MEIZU_M81
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH0_MODE,PMIC_PWM_1);
			pmic_set_register_value(PMIC_ISINK_CH0_STEP,ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL,tr1);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TR2_SEL,tr2);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TF1_SEL,tf1);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TF2_SEL,tf2);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL,ton);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL,toff);
			pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,duty);
			pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,ISINK_1KHZ); //1KHZ
			pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_ON);
			break;
	#else
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH0_MODE,PMIC_PWM_1);
			pmic_set_register_value(PMIC_ISINK_CH0_STEP,ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TR1_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TR2_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TF1_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TF2_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TON_SEL,0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH0_TOFF_SEL,0x03);
			pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,15);
			pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,11);
			//pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_ON);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK1:
			pmic_set_register_value(PMIC_ISINK_CH1_MODE,PMIC_PWM_1);
			pmic_set_register_value(PMIC_ISINK_CH1_STEP,ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TR1_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TR2_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TF1_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TF2_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TON_SEL,0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH1_TOFF_SEL,0x03);
			pmic_set_register_value(PMIC_ISINK_DIM1_DUTY,15);
			pmic_set_register_value(PMIC_ISINK_DIM1_FSEL,11);
			//pmic_set_register_value(PMIC_ISINK_CH1_EN,NLED_ON);
			break;	
		case MT65XX_LED_PMIC_NLED_ISINK2:
			pmic_set_register_value(PMIC_ISINK_CH2_MODE,PMIC_PWM_1);
			pmic_set_register_value(PMIC_ISINK_CH2_STEP,ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TR1_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TR2_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TF1_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TF2_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TON_SEL,0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH2_TOFF_SEL,0x03);
			pmic_set_register_value(PMIC_ISINK_DIM2_DUTY,15);
			pmic_set_register_value(PMIC_ISINK_DIM2_FSEL,11);
			//pmic_set_register_value(PMIC_ISINK_CH2_EN,NLED_ON);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK3:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,PMIC_PWM_1);
			pmic_set_register_value(PMIC_ISINK_CH3_STEP,ISINK_3);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TR1_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TR2_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TF1_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TF2_SEL,0x04);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TON_SEL,0x02);
			pmic_set_register_value(PMIC_ISINK_BREATH3_TOFF_SEL,0x03);
			pmic_set_register_value(PMIC_ISINK_DIM3_DUTY,15);
			pmic_set_register_value(PMIC_ISINK_DIM3_FSEL,11);
			//pmic_set_register_value(PMIC_ISINK_CH3_EN,NLED_ON);
			break;
	#endif //end #ifdef MEIZU_M81
		default:
		break;
	}
  }else {
  	switch(pmic_type){
	#ifdef MEIZU_M81
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH0_MODE,PMIC_PWM_0);
		default:
			break;
	#else
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,PMIC_PWM_0);
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,PMIC_PWM_0);
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,PMIC_PWM_0);
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_CH3_MODE,PMIC_PWM_0);
	#endif //end #ifdef MEIZU_M81
  	}
  }
	return 0;

}
#endif

#define PMIC_PERIOD_NUM 9
/* 100 * period, ex: 0.01 Hz -> 0.01 * 100 = 1 */
int pmic_period_array[] = {250,500,1000,1250,1666,2000,2500,10000};

/* int pmic_freqsel_array[] = {99999, 9999, 4999, 1999, 999, 499, 199, 4, 0}; */
int pmic_freqsel_array[] = {0, 4, 199, 499, 999, 1999, 1999, 1999};

static int find_time_index_pmic(int time_ms)
{
	int i;
	for(i=0;i<PMIC_PERIOD_NUM;i++) {
		if(time_ms<=pmic_period_array[i]) {
			return i;
		} else {
			continue;
		}
	}
	return PMIC_PERIOD_NUM-1;
}

int mt_led_blink_pmic(enum mt65xx_led_pmic pmic_type, struct nled_setting *led)
{
	int time_index = 0;
	int duty = 0;
	LEDS_DEBUG("[LED]led_blink_pmic: pmic_type=%d\n", pmic_type);  
	
	if((pmic_type != MT65XX_LED_PMIC_NLED_ISINK0 && pmic_type!= MT65XX_LED_PMIC_NLED_ISINK1) || 
		led->nled_mode != NLED_BLINK) {
		return -1;
	}
				
	LEDS_DEBUG("[LED]LED blink on time = %d offtime = %d\n",led->blink_on_time,led->blink_off_time);
	time_index = find_time_index_pmic(led->blink_on_time + led->blink_off_time);
	LEDS_DEBUG("[LED]LED index is %d  freqsel=%d\n", time_index, pmic_freqsel_array[time_index]);
	duty=32*led->blink_on_time/(led->blink_on_time + led->blink_off_time);
	//pmic_set_register_value(PMIC_RG_G_DRV_2M_CK_PDN(0X0); // DISABLE POWER DOWN ,Indicator no need)
    pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down  
	switch(pmic_type){
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN,0);
			pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL,0);
			pmic_set_register_value(PMIC_ISINK_CH0_MODE,PMIC_PWM_0);
			pmic_set_register_value(PMIC_ISINK_CH0_STEP,ISINK_3);//16mA
			pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,duty);
			pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,pmic_freqsel_array[time_index]);			
			pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_ON);
			break;
		case MT65XX_LED_PMIC_NLED_ISINK1:
			pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_PDN,0);
			pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_CKSEL,0);
			pmic_set_register_value(PMIC_ISINK_CH1_MODE,PMIC_PWM_0);
			pmic_set_register_value(PMIC_ISINK_CH1_STEP,ISINK_3);//16mA
			pmic_set_register_value(PMIC_ISINK_DIM1_DUTY,duty);
			pmic_set_register_value(PMIC_ISINK_DIM1_FSEL,pmic_freqsel_array[time_index]);		
			pmic_set_register_value(PMIC_ISINK_CH1_EN,NLED_ON);
			break;	
		default:
			break;
	}
	return 0;
}


int mt_backlight_set_pwm(int pwm_num, u32 level, u32 div, struct PWM_config *config_data)
{
	struct pwm_spec_config pwm_setting;
	unsigned int BacklightLevelSupport = Cust_GetBacklightLevelSupport_byPWM();
	pwm_setting.pwm_no = pwm_num;
	
	if (BacklightLevelSupport == BACKLIGHT_LEVEL_PWM_256_SUPPORT)
		pwm_setting.mode = PWM_MODE_OLD;
	else
		pwm_setting.mode = PWM_MODE_FIFO;	/* New mode fifo and periodical mode */

	pwm_setting.pmic_pad = config_data->pmic_pad;
		
	if(config_data->div)
	{
		pwm_setting.clk_div = config_data->div;
		backlight_PWM_div_hal = config_data->div;
	}
	else
		pwm_setting.clk_div = div;
	
		if(BacklightLevelSupport== BACKLIGHT_LEVEL_PWM_256_SUPPORT)
	{
		if(config_data->clock_source)
		{
			pwm_setting.clk_src = PWM_CLK_OLD_MODE_BLOCK;
		}
		else
		{
			pwm_setting.clk_src = PWM_CLK_OLD_MODE_32K;	 // actually. it's block/1625 = 26M/1625 = 16KHz @ MT6571
		}

		pwm_setting.PWM_MODE_OLD_REGS.IDLE_VALUE = 0;
		pwm_setting.PWM_MODE_OLD_REGS.GUARD_VALUE = 0;
		pwm_setting.PWM_MODE_OLD_REGS.GDURATION = 0;
		pwm_setting.PWM_MODE_OLD_REGS.WAVE_NUM = 0;
		pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH = 255; // 256 level
		pwm_setting.PWM_MODE_OLD_REGS.THRESH = level;

		LEDS_DEBUG("[LEDS][%d]backlight_set_pwm:duty is %d/%d\n", BacklightLevelSupport, level, pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
		LEDS_DEBUG("[LEDS][%d]backlight_set_pwm:clk_src/div is %d%d\n", BacklightLevelSupport, pwm_setting.clk_src, pwm_setting.clk_div);
		if(level >0 && level < 256)
		{
			pwm_set_spec_config(&pwm_setting);
			LEDS_DEBUG("[LEDS][%d]backlight_set_pwm: old mode: thres/data_width is %d/%d\n", BacklightLevelSupport, pwm_setting.PWM_MODE_OLD_REGS.THRESH, pwm_setting.PWM_MODE_OLD_REGS.DATA_WIDTH);
		}
		else
		{
			LEDS_DEBUG("[LEDS][%d]Error level in backlight\n", BacklightLevelSupport);
			mt_pwm_disable(pwm_setting.pwm_no, config_data->pmic_pad);
		}
		return 0;

	}
	else
	{
		if(config_data->clock_source)
		{
			pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
		}
		else
		{
			pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK_DIV_BY_1625;
		}

		if(config_data->High_duration && config_data->low_duration)
		{
			pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = config_data->High_duration;
			pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = pwm_setting.PWM_MODE_FIFO_REGS.HDURATION;
		}
		else
		{
			pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 4;
			pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 4;
		}

		pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = 0;
		pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
		pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 31;
		pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = (pwm_setting.PWM_MODE_FIFO_REGS.HDURATION + 1) * 32 - 1;
		pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM = 0;

		LEDS_DEBUG("[LEDS]backlight_set_pwm:duty is %d\n", level);
		LEDS_DEBUG("[LEDS]backlight_set_pwm:clk_src/div/high/low is %d%d%d%d\n", pwm_setting.clk_src, pwm_setting.clk_div, pwm_setting.PWM_MODE_FIFO_REGS.HDURATION, pwm_setting.PWM_MODE_FIFO_REGS.LDURATION);

		if(level > 0 && level <= 32)
		{
			pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 0;
			pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = (1 << level) - 1;
			pwm_set_spec_config(&pwm_setting);
		}else if(level > 32 && level <= 64)
		{
			pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = 1;
			level -= 32;
			pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = (1 << level) - 1 ;
			pwm_set_spec_config(&pwm_setting);
		}else
		{
			LEDS_DEBUG("[LEDS]Error level in backlight\n");
			mt_pwm_disable(pwm_setting.pwm_no, config_data->pmic_pad);
		}

		return 0;

	}
}
void mt_led_pwm_disable(int pwm_num)
{
	struct cust_mt65xx_led *cust_led_list = get_cust_led_list();
	mt_pwm_disable(pwm_num, cust_led_list->config_data.pmic_pad);
}

void mt_backlight_set_pwm_duty(int pwm_num, u32 level, u32 div, struct PWM_config *config_data)
{
	mt_backlight_set_pwm(pwm_num, level, div, config_data);
}

void mt_backlight_set_pwm_div(int pwm_num, u32 level, u32 div, struct PWM_config *config_data)
{
	mt_backlight_set_pwm(pwm_num, level, div, config_data);
}

void mt_backlight_get_pwm_fsel(unsigned int bl_div, unsigned int *bl_frequency)
{

}

void mt_store_pwm_register(unsigned int addr, unsigned int value)
{

}

unsigned int mt_show_pwm_register(unsigned int addr)
{
	return 0;
}


#ifdef MEIZU_M81

struct _arr_cur_map
{
	unsigned char cur;
	unsigned char duty;
};

static struct _arr_cur_map button_current_mapping[] =
{
	{	0,	0, 	},//1
	{	1,	0,	},//2
	{	0,	1, 	},//3
	{	2,	0,	},//4
	{	0,	2, 	},//5
	{	3,	0,	},//6
	{	1,	1,	},//7
	{	0,	3, 	},//8
	{	4,	0,	},//9
	{	0,	4, 	},//10
	{	2,	1,	},//11
	{	1,	2, 	},//12
	{	0,	5, 	},//13
	{	0,	6, 	},//14
	{	3,	1,	},//15
	{	1,	3, 	},//16
	{	0,	7, 	},//17
	{	2,	2, 	},//18
	{	0,	8, 	},//19
	{	4,	1,	},//20
	{	1,	4, 	},//21
	{	0,	9, 	},//22
	{	0,	10,	},//23
	{	3,	2,	},//24
	{	2,	3,	},//25
	{	1,	5, 	},//26
	{	0,	11,	},//27
	{	0,	12,	},//28
	{	1,	6, 	},//29
	{	0,	13,	},//30
	{	4,	2,	},//31
	{	2,	4,	},//32
	{	0,	14,	},//33
	{	3,	3,	},//34
	{	1,	7,	},//35
	{	0,	15,	},//36
	{	0,	16,	},//37
	{	1,	8,	},//38
	{	0,	17,	},//39
	{	0,	18,	},//40
	{	1,	9,	},//41
	{	2,	6,	},//42
	{	0,	20,	},//43
	{	1,	10,	},//44
	{	0,	22,	},//45
	{	1,	11,	},//46
	{	4,	4,	},//47
	{	0,	24,	},//48
	{	0,	25,	},//49
	{	2,	8, 	},//50
	{	3,	6,	},//51
	{	4,	5,	},//52
	{	1,	14,	},//53
	{	3,	7,	},//54
	{	1,	15,	},//55
	{	2,	10,	},//56
	{	4,	6,	},//57
	{	3,	8,	},//58
	{	1,	17,	},//59
	{	1,	18,	},//60
	{	2,	12,	},//61
	{	3,	9,	},//62
	{	1,	19,	},//63
	{	1,	20,	},//64
	{	3,	10,	},//65
	{	2,	14,	},//66
	{	1,	22,	},//67
	{	2,	15,	},//68
	{	4,	9,	},//69
	{	1,	24,	},//70
	{	2,	16,	},//71
	{	1,	25,	},//72
	{	4,	10,	},//73
	{	2,	18,	},//74
	{	4,	11,	},//75
	{	2,	19,	},//76
	{	2,	20,	},//77
	{	4,	13,	},//78
	{	4,	14,	},//79
	{	4,	15,	},//80
};

/* The max length of array button_current_mapping[] */
#define LEVEL_RANGE	(ARRAY_SIZE(button_current_mapping))

int mt_brightness_set_pmic_level(enum mt65xx_led_pmic pmic_type, u32 level)
{
	unsigned char duty,cur;

	LEDS_DEBUG("[LED]%s:PMIC#%d:%d\n", __func__, pmic_type, level);

	// level: 1 ~ LEVEL_RANGE
	if(level == ERROR_BL_LEVEL)
		level = 255;

	if(level >= 255)
	{
		level = LEVEL_RANGE;
	}
	else
	{
		level = (level * LEVEL_RANGE / 255) + 1;
	}

	// get the duty and current of level from array
	duty = button_current_mapping[level-1].duty;
	cur = button_current_mapping[level-1].cur;

	LEDS_DEBUG("[LED]Level Mapping = %d\n", level);
	LEDS_DEBUG("[LED]ISINK DIM Duty = %d\n", duty);
	LEDS_DEBUG("[LED]ISINK Current = %d\n", cur);

	switch(pmic_type)
	{
		case MT65XX_LED_PMIC_NLED_ISINK0:
			pmic_set_register_value(PMIC_ISINK_DIM0_DUTY, duty);
			pmic_set_register_value(PMIC_ISINK_CH0_STEP, cur);//isink current
			break;
		default:
			break;
	}

	return 0;
}
#endif //end #ifdef MEIZU_M81

int mt_brightness_set_pmic(enum mt65xx_led_pmic pmic_type, u32 level, u32 div)
{
	static bool first_time = true;

	LEDS_DEBUG("[LED]PMIC#%d:%d\n", pmic_type, level);
	mutex_lock(&leds_pmic_mutex);
		if(pmic_type == MT65XX_LED_PMIC_NLED_ISINK0)
		{
			if((button_flag_isink0==0) && (first_time == true)) {//button flag ==0, means this ISINK is not for button backlight
				if(button_flag_isink1==0)
				pmic_set_register_value(PMIC_ISINK_CH1_EN,NLED_OFF);  //sw workround for sync leds status 
				if(button_flag_isink2==0)
				pmic_set_register_value(PMIC_ISINK_CH2_EN,NLED_OFF);
				if(button_flag_isink3==0)
				pmic_set_register_value(PMIC_ISINK_CH3_EN,NLED_OFF);
				first_time = false;
			}
			pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down  
			pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN,0);
			pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL,0);
			pmic_set_register_value(PMIC_ISINK_CH0_MODE,PMIC_PWM_0);
#ifndef MEIZU_M81
			pmic_set_register_value(PMIC_ISINK_CH0_STEP,ISINK_3);//16mA
			pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,15);
#endif
			pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,ISINK_1KHZ);//1KHz
#ifdef MEIZU_M81
			button_brightness_hal = level;
#endif
			if (level) 
			{
#ifdef MEIZU_M81
				mt_brightness_set_pmic_level(MT65XX_LED_PMIC_NLED_ISINK0, level);
#endif
				pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_ON);
				
			}
			else 
			{
				pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_OFF);
			}
			mutex_unlock(&leds_pmic_mutex);
			return 0;
		}
		else if(pmic_type == MT65XX_LED_PMIC_NLED_ISINK1)
		{
			if((button_flag_isink1==0) && (first_time == true)) {//button flag ==0, means this ISINK is not for button backlight
				if(button_flag_isink0==0)
				pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_OFF);  //sw workround for sync leds status
				if(button_flag_isink2==0)
				pmic_set_register_value(PMIC_ISINK_CH2_EN,NLED_OFF);
				if(button_flag_isink3==0)
				pmic_set_register_value(PMIC_ISINK_CH3_EN,NLED_OFF);
				first_time = false;
			}	
			pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down  
			pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_PDN,0);
			pmic_set_register_value(PMIC_RG_DRV_ISINK1_CK_CKSEL,0);
			pmic_set_register_value(PMIC_ISINK_CH1_MODE,PMIC_PWM_0);
			pmic_set_register_value(PMIC_ISINK_CH1_STEP,ISINK_3);//16mA
			pmic_set_register_value(PMIC_ISINK_DIM1_DUTY,15);
			pmic_set_register_value(PMIC_ISINK_DIM1_FSEL,ISINK_1KHZ);//1KHz
			if (level) 
			{
				pmic_set_register_value(PMIC_ISINK_CH1_EN,NLED_ON);
			}
			else 
			{
				pmic_set_register_value(PMIC_ISINK_CH1_EN,NLED_OFF);
			}
			mutex_unlock(&leds_pmic_mutex);
			return 0;
		}
		mutex_unlock(&leds_pmic_mutex);	
		return -1;
}

int mt_brightness_set_pmic_duty_store(u32 level, u32 div)
{
	return -1;
}
#ifdef MEIZU_M81
static int cust_last_level = 0;
#endif
int mt_mt65xx_led_set_cust(struct cust_mt65xx_led *cust, int level)
{
	struct nled_setting led_tmp_setting = {0,0,0};
	int tmp_level = level;
	static bool button_flag = false;
	static int last_level = 0, same_level_cnt = 0;
	unsigned int BacklightLevelSupport = Cust_GetBacklightLevelSupport_byPWM();
	/* Mark out since the level is already cliped before sending in */
	/*
		if (level > LED_FULL)
			level = LED_FULL;
		else if (level < 0)
			level = 0;
	*/	

	if (last_level==level)
	{
		if(same_level_cnt < 1)
		{ // To avoid the upper layer (e.g. AAL) to print too many log
		    //LEDS_DEBUG("mt65xx_leds_set_cust: set brightness, name:%s, mode:%d, level:%d\n", 
			//cust->name, cust->mode, level);
		}
	}
	else
	{
		if(same_level_cnt>1)
		{
			//LEDS_DEBUG("mt65xx_leds_set_cust: set brightness, name:%s, mode:%d, level:%d, last_same_cnt: %d\n", 
			//cust->name, cust->mode, level, same_level_cnt);
			same_level_cnt=0;
		}
		else
		{
			//LEDS_DEBUG("mt65xx_leds_set_cust: set brightness, name:%s, mode:%d, level:%d\n", 
			//cust->name, cust->mode, level);
		}
	}
	last_level = level;
	same_level_cnt++;

	switch (cust->mode) {
		
		case MT65XX_LED_MODE_PWM:
			if(strcmp(cust->name,"lcd-backlight") == 0)
			{
				bl_brightness_hal = level;
				if(level == 0)
				{
					mt_pwm_disable(cust->data, cust->config_data.pmic_pad);
					
				}else
				{
					if (BacklightLevelSupport == BACKLIGHT_LEVEL_PWM_256_SUPPORT)
						level = brightness_mapping(tmp_level);
					else
						level = brightness_mapto64(tmp_level);	
					mt_backlight_set_pwm(cust->data, level, bl_div_hal,&cust->config_data);
				}
                bl_duty_hal = level;	
				
			}else
			{
				if(level == 0)
				{
					led_tmp_setting.nled_mode = NLED_OFF;
					mt_led_set_pwm(cust->data,&led_tmp_setting);
					mt_pwm_disable(cust->data, cust->config_data.pmic_pad);
				}else
				{
					led_tmp_setting.nled_mode = NLED_ON;
					mt_led_set_pwm(cust->data,&led_tmp_setting);
				}
			}
			return 1;
          
		case MT65XX_LED_MODE_GPIO:
			LEDS_DEBUG("brightness_set_cust:go GPIO mode!!!!!\n");
			return ((cust_set_brightness)(cust->data))(level);
              
		case MT65XX_LED_MODE_PMIC:
			//for button baclight used SINK channel, when set button ISINK, don't do disable other ISINK channel
			if((strcmp(cust->name,"button-backlight") == 0)) {
				if(button_flag==false) {
					switch (cust->data) {
						case MT65XX_LED_PMIC_NLED_ISINK0:
							button_flag_isink0 = 1;
							break;
						case MT65XX_LED_PMIC_NLED_ISINK1:
							button_flag_isink1 = 1;
							break;
						case MT65XX_LED_PMIC_NLED_ISINK2:
							button_flag_isink2 = 1;
							break;
						case MT65XX_LED_PMIC_NLED_ISINK3:
							button_flag_isink3 = 1;
							break;
						default:
							break;
					}
					button_flag=true;
				}
			}					
			return mt_brightness_set_pmic(cust->data, level, bl_div_hal);
            
		case MT65XX_LED_MODE_CUST_LCM:
		if (strcmp(cust->name, "lcd-backlight") == 0) {
			    bl_brightness_hal = level;
            }
			LEDS_DEBUG("brightness_set_cust:backlight control by LCM\n");
			return ((cust_brightness_set)(cust->data))(level, bl_div_hal);

		case MT65XX_LED_MODE_CUST_BLS_PWM:
		if (strcmp(cust->name, "lcd-backlight") == 0) {
				bl_brightness_hal = level;
#ifdef MEIZU_M81
				if(level == 0)
				{
					cust_last_level = level;
					Cust_SetBacklight(level);
				}
				else
				{
					//printk(KERN_EMERG"MT65XX_LED_MODE_CUST_BLS_PWM level=%d \n",level);
						Cust_SetBacklight(160);
					cust_last_level = level;
				}
#endif
			}
			return ((cust_set_brightness)(cust->data))(level);
            
		case MT65XX_LED_MODE_NONE:
		default:
			break;
	}
	return -1;
}

void mt_mt65xx_led_work(struct work_struct *work)
{
	struct mt65xx_led_data *led_data =
		container_of(work, struct mt65xx_led_data, work);

	LEDS_DEBUG("[LED]%s:%d\n", led_data->cust.name, led_data->level);
	mutex_lock(&leds_mutex);
	mt_mt65xx_led_set_cust(&led_data->cust, led_data->level);
	mutex_unlock(&leds_mutex);;
}

#ifdef MEIZU_M81
void mz_set_oneshot_blink(void)
{
	unsigned int i = 0;
	unsigned int level_hal = button_brightness_hal;

	LEDS_DEBUG("[LED]led_isink0_tigger_pmic: level_from=%u\n", level_hal);

	if(level_hal < 34)
		level_hal = 34;

	pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable power down
	pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN,0);
	pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL,0);
	pmic_set_register_value(PMIC_ISINK_CH0_MODE,PMIC_PWM_0);
	pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,ISINK_1KHZ); //1KHZ
	pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_ON);

	/*************first blink**************/
	// from level_hal to 255 and step=3
	for(i = level_hal; i <= 255; i+=3)
	{
		mt_brightness_set_pmic_level(MT65XX_LED_PMIC_NLED_ISINK0, i);
		usleep_range(500, 2000);
	}
	// from 255 to level_hal and step=3
	for(i = 255; i >= level_hal; i-=3)
	{
		mt_brightness_set_pmic_level(MT65XX_LED_PMIC_NLED_ISINK0, i);
		usleep_range(500, 2000);
	}
	/*************Second blink**************/
	// from level_hal to 255 and step=3
	for(i = level_hal; i <= 255; i+=3)
	{
		mt_brightness_set_pmic_level(MT65XX_LED_PMIC_NLED_ISINK0, i);
		usleep_range(500, 2000);
	}
	// from 255 to level_hal and step=3
	for(i = 255; i >= level_hal; i-=3)
	{
		mt_brightness_set_pmic_level(MT65XX_LED_PMIC_NLED_ISINK0, i);
		usleep_range(500, 2000);
	}

	// if LED is off before oneshot blink, should turn off LED
	// if LED is on before oneshot blink, set LED brightness as same as primary.
	if (!button_brightness_hal)
		pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_OFF);
	else
		mt_brightness_set_pmic_level(MT65XX_LED_PMIC_NLED_ISINK0, button_brightness_hal);
}

void mz_led_oneshot_work(struct work_struct *work)
{
	mutex_lock(&led_oneshot_mutex);
	mz_set_oneshot_blink();
	mutex_unlock(&led_oneshot_mutex);
}
#endif //end #ifdef MEIZU_M81

static mz_cust_last_level = 0;

void mt_mt65xx_led_set(struct led_classdev *led_cdev, enum led_brightness level)
{
	struct mt65xx_led_data *led_data =
		container_of(led_cdev, struct mt65xx_led_data, cdev);
	//unsigned long flags;
	//spin_lock_irqsave(&leds_lock, flags);
	
#ifdef CONFIG_MTK_AAL_SUPPORT
	if(led_data->level != level)
	{
		led_data->level = level;
		if(strcmp(led_data->cust.name,"lcd-backlight") != 0)
		{
			LEDS_DEBUG("[LED]Set NLED directly %d at time %lu\n",led_data->level,jiffies);
			schedule_work(&led_data->work);				
		}
		else
		{
			if( level != 0 && level*CONFIG_LIGHTNESS_MAPPING_VALUE < 255 )
			{
				level = 1;
			} else {
				level = (level*CONFIG_LIGHTNESS_MAPPING_VALUE)/255;
			}
			LEDS_DEBUG("[LED]Set Backlight directly %d at time %lu, mappping level is %d \n",led_data->level,jiffies, level);
			//mt_mt65xx_led_set_cust(&led_data->cust, led_data->level);	
			disp_aal_notify_backlight_changed( (((1 << MT_LED_INTERNAL_LEVEL_BIT_CNT) - 1)*level + 127)/255 );
		}
	}
#else						
	// do something only when level is changed
	if(led_data->level != level)
	{
		led_data->level = level;
		if(strcmp(led_data->cust.name,"lcd-backlight") != 0)
		{
			LEDS_DEBUG("[LED]Set NLED directly %d at time %lu\n",led_data->level,jiffies);
				schedule_work(&led_data->work);
		}
		else
		{
			if( level != 0 && level*CONFIG_LIGHTNESS_MAPPING_VALUE < 255 )
			{
				level = 1;
			} else {
				level = (level*CONFIG_LIGHTNESS_MAPPING_VALUE)/255;
			}
			LEDS_DEBUG("[LED]Set Backlight directly %d at time %lu, mappping level is %d \n",led_data->level,jiffies, level);
			if(MT65XX_LED_MODE_CUST_BLS_PWM == led_data->cust.mode)
			{
				//mt_mt65xx_led_set_cust(&led_data->cust, ((((1 << MT_LED_INTERNAL_LEVEL_BIT_CNT) - 1)*level + 127)/255));
				{
					int tmp_level = 0;
					int dst_level = 0;
					int levle_flag = 0;
					int cust_level = ((((1 << MT_LED_INTERNAL_LEVEL_BIT_CNT) - 1)*level + 127)/255);

					if (level != 0)
					{
						if (mz_cust_last_level > level)
						{
							tmp_level = cust_level;
							if (level == 1)
							{
								dst_level = cust_level-3;
							}
							else
							{
								dst_level = cust_level-4;
							}
							levle_flag = 0;
						}
						else
						{
							if (level == 1)
							{
								tmp_level = cust_level-3;
							}
							else
							{
								tmp_level = cust_level-4;
							}
							dst_level = cust_level;
							levle_flag = 1;
						}
						do
						{
							mt_mt65xx_led_set_cust(&led_data->cust, tmp_level);
							//mdelay(10);
							if (levle_flag == 0)
							{
								tmp_level--;
							}
							else
							{
								tmp_level++;
							}
						}while(tmp_level != dst_level);
					}
					else
					{
						mt_mt65xx_led_set_cust(&led_data->cust, ((((1 << MT_LED_INTERNAL_LEVEL_BIT_CNT) - 1)*level + 127)/255));
					}
					mz_cust_last_level = level;
				}
			}
			else
			{
				mt_mt65xx_led_set_cust(&led_data->cust, level);	
		}
	}
	}
	//spin_unlock_irqrestore(&leds_lock, flags);
#endif
//    if(0!=aee_kernel_Powerkey_is_press())
//		aee_kernel_wdt_kick_Powkey_api("mt_mt65xx_led_set",WDT_SETBY_Backlight); 
}

int  mt_mt65xx_blink_set(struct led_classdev *led_cdev,
			     unsigned long *delay_on,
			     unsigned long *delay_off)
{
	struct mt65xx_led_data *led_data =
		container_of(led_cdev, struct mt65xx_led_data, cdev);
	static int got_wake_lock = 0;
	struct nled_setting nled_tmp_setting = {0,0,0};

	// only allow software blink when delay_on or delay_off changed
	if (*delay_on != led_data->delay_on || *delay_off != led_data->delay_off) {
		led_data->delay_on = *delay_on;
		led_data->delay_off = *delay_off;
		if (led_data->delay_on && led_data->delay_off) { // enable blink
			led_data->level = 255; // when enable blink  then to set the level  (255)
			//AP PWM all support OLD mode 
			if(led_data->cust.mode == MT65XX_LED_MODE_PWM)
			{
				nled_tmp_setting.nled_mode = NLED_BLINK;
				nled_tmp_setting.blink_off_time = led_data->delay_off;
				nled_tmp_setting.blink_on_time = led_data->delay_on;
				mt_led_set_pwm(led_data->cust.data,&nled_tmp_setting);
				return 0;
			}
			else if((led_data->cust.mode == MT65XX_LED_MODE_PMIC) && (led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK0
				|| led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK1 || led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK2
				 || led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK3))
			{
				nled_tmp_setting.nled_mode = NLED_BLINK;
				nled_tmp_setting.blink_off_time = led_data->delay_off;
				nled_tmp_setting.blink_on_time = led_data->delay_on;
#ifndef MEIZU_M81
				mt_led_blink_pmic(led_data->cust.data, &nled_tmp_setting);
#else
				led_switch_breath_pmic(led_data->cust.data, &nled_tmp_setting, ISINK_BREATH_MODE);//set led: breath mode
				//clear delay_on and delay_off
				led_data->delay_on = 0;
				led_data->delay_off = 0;
#endif
				return 0;
			}				
			else if (!got_wake_lock) {
				wake_lock(&leds_suspend_lock);
				got_wake_lock = 1;
			}
		}
		else if (!led_data->delay_on && !led_data->delay_off) { // disable blink
			//AP PWM all support OLD mode 
			if(led_data->cust.mode == MT65XX_LED_MODE_PWM)
			{
				nled_tmp_setting.nled_mode = NLED_OFF;
				mt_led_set_pwm(led_data->cust.data,&nled_tmp_setting);
				return 0;
			}
			else if((led_data->cust.mode == MT65XX_LED_MODE_PMIC) && (led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK0
				|| led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK1 || led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK2
				 || led_data->cust.data == MT65XX_LED_PMIC_NLED_ISINK3))
			{
				mt_brightness_set_pmic(led_data->cust.data, 0, 0);
				return 0;
			}		
			else if (got_wake_lock) {
				wake_unlock(&leds_suspend_lock);
				got_wake_lock = 0;
			}
		}
		return -1;
	}

	// delay_on and delay_off are not changed
	return 0;
}




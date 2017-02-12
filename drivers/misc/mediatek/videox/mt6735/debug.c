#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <mach/mt_typedefs.h>
#include <mach/m4u.h>
#include "disp_drv_log.h"
#include "mtkfb.h"
#include "debug.h"
#include "lcm_drv.h"
#include "ddp_ovl.h"
#include "ddp_path.h"
#include "ddp_reg.h"
#include "primary_display.h"
#include "display_recorder.h"
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/mt_clkmgr.h>
#include "mtkfb_fence.h"
#include "ddp_manager.h"

struct MTKFB_MMP_Events_t MTKFB_MMP_Events;

extern LCM_DRIVER *lcm_drv;
extern unsigned int EnableVSyncLog;
extern unsigned int gTriggerDispMode;  // 0: normal, 1: lcd only, 2: none of lcd and lcm

#define MTKFB_DEBUG_FS_CAPTURE_LAYER_CONTENT_SUPPORT

// ---------------------------------------------------------------------------
//  External variable declarations
// ---------------------------------------------------------------------------

extern long tpd_last_down_time;
extern int  tpd_start_profiling;
extern void mtkfb_log_enable(int enable);
extern void disp_log_enable(int enable);
extern void mtkfb_vsync_log_enable(int enable);
extern void mtkfb_capture_fb_only(bool enable);
extern void esd_recovery_pause(BOOL en);
extern int mtkfb_set_backlight_mode(unsigned int mode);
extern void mtkfb_pan_disp_test(void);
extern void mtkfb_show_sem_cnt(void);
extern void mtkfb_hang_test(bool en);
extern void mtkfb_switch_normal_to_factory(void);
extern void mtkfb_switch_factory_to_normal(void);
extern int mtkfb_get_debug_state(char* stringbuf, int buf_len);
extern unsigned int mtkfb_fm_auto_test(void);

extern unsigned int gCaptureLayerEnable;
extern unsigned int gCaptureLayerDownX;
extern unsigned int gCaptureLayerDownY;

extern unsigned int gCaptureOvlThreadEnable;
extern unsigned int gCaptureOvlDownX;
extern unsigned int gCaptureOvlDownY;
extern struct task_struct *captureovl_task;

extern unsigned int gCaptureFBEnable;
extern unsigned int gCaptureFBDownX;
extern unsigned int gCaptureFBDownY;
extern unsigned int gCaptureFBPeriod;
extern struct task_struct *capturefb_task;
extern wait_queue_head_t gCaptureFBWQ;

extern unsigned int gCapturePriLayerEnable;
extern unsigned int gCaptureWdmaLayerEnable;
extern unsigned int gCapturePriLayerDownX;
extern unsigned int gCapturePriLayerDownY;
extern unsigned int gCapturePriLayerNum;
#ifdef MTKFB_DEBUG_FS_CAPTURE_LAYER_CONTENT_SUPPORT
struct dentry *mtkfb_layer_dbgfs[DDP_OVL_LAYER_MUN];

extern OVL_CONFIG_STRUCT cached_layer_config[DDP_OVL_LAYER_MUN];

typedef struct {
    UINT32 layer_index;
    unsigned long working_buf;
    UINT32 working_size;
} MTKFB_LAYER_DBG_OPTIONS;

MTKFB_LAYER_DBG_OPTIONS mtkfb_layer_dbg_opt[DDP_OVL_LAYER_MUN];

#endif    
extern LCM_DRIVER *lcm_drv;
// ---------------------------------------------------------------------------
//  Debug Options
// ---------------------------------------------------------------------------

static const long int DEFAULT_LOG_FPS_WND_SIZE = 30;

typedef struct {
    unsigned int en_fps_log;
    unsigned int en_touch_latency_log;
    unsigned int log_fps_wnd_size;
    unsigned int force_dis_layers;
} DBG_OPTIONS;

static DBG_OPTIONS dbg_opt = {0};
static bool enable_ovl1_to_mem = true;


static char STR_HELP[] =
    "\n"
    "USAGE\n"
    "        echo [ACTION]... > /d/mtkfb\n"
    "\n"
    "ACTION\n"
	"        mtkfblog:[on|off]\n"
	"             enable/disable [MTKFB] log\n"
	"\n"
	"        displog:[on|off]\n"
	"             enable/disable [DISP] log\n"
	"\n"
	"        mtkfb_vsynclog:[on|off]\n"
	"             enable/disable [VSYNC] log\n"
	"\n"
	"        log:[on|off]\n"
	"             enable/disable above all log\n"
	"\n"
    "        fps:[on|off]\n"
    "             enable fps and lcd update time log\n"
	"\n"
    "        tl:[on|off]\n"
    "             enable touch latency log\n"
    "\n"
    "        layer\n"
    "             dump lcd layer information\n"
    "\n"
    "        suspend\n"
    "             enter suspend mode\n"
    "\n"
    "        resume\n"
    "             leave suspend mode\n"
    "\n"
    "        lcm:[on|off|init]\n"
    "             power on/off lcm\n"
    "\n"
    "        cabc:[ui|mov|still]\n"
    "             cabc mode, UI/Moving picture/Still picture\n"
    "\n"
    "        lcd:[on|off]\n"
    "             power on/off display engine\n"
    "\n"
    "        te:[on|off]\n"
    "             turn on/off tearing-free control\n"
    "\n"
    "        tv:[on|off]\n"
    "             turn on/off tv-out\n"
    "\n"
    "        tvsys:[ntsc|pal]\n"
    "             switch tv system\n"
    "\n"
    "        reg:[lcd|dpi|dsi|tvc|tve]\n"
    "             dump hw register values\n"
    "\n"
    "        regw:addr=val\n"
    "             write hw register\n"
    "\n"
    "        regr:addr\n"
    "             read hw register\n"
    "\n"     
    "       cpfbonly:[on|off]\n"
    "             capture UI layer only on/off\n"
    "\n"     
    "       esd:[on|off]\n"
    "             esd kthread on/off\n"
    "       HQA:[NormalToFactory|FactoryToNormal]\n"
    "             for HQA requirement\n"
    "\n"
    "       cmd:[value]\n"
    "             set cmd\n"
    "\n"     
    "       mmp\n"
    "             Register MMProfile events\n"
	"\n"
	"       dump_fb:[on|off[,down_sample_x[,down_sample_y,[delay]]]]\n"
	"             Start/end to capture framebuffer every delay(ms)\n"
	"\n"
	"       dump_ovl:[on|off[,down_sample_x[,down_sample_y]]]\n"
	"             Start to capture OVL only once\n"
	"\n"
	"       dump_layer:[on|off[,down_sample_x[,down_sample_y]][,layer(0:L0,1:L1,2:L2,3:L3,4:L0-3)]\n"
	"             Start/end to capture current enabled OVL layer every frame\n"
    ;


// ---------------------------------------------------------------------------
//  Information Dump Routines
// ---------------------------------------------------------------------------

void init_mtkfb_mmp_events(void)
{
    if (MTKFB_MMP_Events.MTKFB == 0)
    {
        MTKFB_MMP_Events.MTKFB = MMProfileRegisterEvent(MMP_RootEvent, "MTKFB");
        MTKFB_MMP_Events.PanDisplay = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "PanDisplay");
        MTKFB_MMP_Events.CreateSyncTimeline = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "CreateSyncTimeline");
        MTKFB_MMP_Events.SetOverlayLayer = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "SetOverlayLayer");
        MTKFB_MMP_Events.SetOverlayLayers = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "SetOverlayLayers");
        MTKFB_MMP_Events.SetMultipleLayers = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "SetMultipleLayers");
        MTKFB_MMP_Events.CreateSyncFence = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "CreateSyncFence");
        MTKFB_MMP_Events.IncSyncTimeline = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "IncSyncTimeline");
        MTKFB_MMP_Events.SignalSyncFence = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "SignalSyncFence");
        MTKFB_MMP_Events.TrigOverlayOut = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "TrigOverlayOut");
        MTKFB_MMP_Events.UpdateScreenImpl = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "UpdateScreenImpl");
        MTKFB_MMP_Events.VSync = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "VSync");
        MTKFB_MMP_Events.UpdateConfig = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "UpdateConfig");
        MTKFB_MMP_Events.EsdCheck = MMProfileRegisterEvent(MTKFB_MMP_Events.UpdateConfig, "EsdCheck");
        MTKFB_MMP_Events.ConfigOVL = MMProfileRegisterEvent(MTKFB_MMP_Events.UpdateConfig, "ConfigOVL");
        MTKFB_MMP_Events.ConfigAAL = MMProfileRegisterEvent(MTKFB_MMP_Events.UpdateConfig, "ConfigAAL");
        MTKFB_MMP_Events.ConfigMemOut = MMProfileRegisterEvent(MTKFB_MMP_Events.UpdateConfig, "ConfigMemOut");
        MTKFB_MMP_Events.ScreenUpdate = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "ScreenUpdate");
        MTKFB_MMP_Events.CaptureFramebuffer = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "CaptureFB");
        MTKFB_MMP_Events.RegUpdate = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "RegUpdate");
        MTKFB_MMP_Events.EarlySuspend = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "EarlySuspend");
        MTKFB_MMP_Events.DispDone = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "DispDone");
        MTKFB_MMP_Events.DSICmd = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "DSICmd");
        MTKFB_MMP_Events.DSIIRQ = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "DSIIrq");
        MTKFB_MMP_Events.WaitVSync = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "WaitVSync");
        MTKFB_MMP_Events.LayerDump = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "LayerDump");
        MTKFB_MMP_Events.Layer[0] = MMProfileRegisterEvent(MTKFB_MMP_Events.LayerDump, "Layer0");
        MTKFB_MMP_Events.Layer[1] = MMProfileRegisterEvent(MTKFB_MMP_Events.LayerDump, "Layer1");
        MTKFB_MMP_Events.Layer[2] = MMProfileRegisterEvent(MTKFB_MMP_Events.LayerDump, "Layer2");
        MTKFB_MMP_Events.Layer[3] = MMProfileRegisterEvent(MTKFB_MMP_Events.LayerDump, "Layer3");
        MTKFB_MMP_Events.OvlDump = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "OvlDump");
        MTKFB_MMP_Events.FBDump = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "FBDump");
        MTKFB_MMP_Events.DSIRead = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "DSIRead");
        MTKFB_MMP_Events.GetLayerInfo = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "GetLayerInfo");
        MTKFB_MMP_Events.LayerInfo[0] = MMProfileRegisterEvent(MTKFB_MMP_Events.GetLayerInfo, "LayerInfo0");
        MTKFB_MMP_Events.LayerInfo[1] = MMProfileRegisterEvent(MTKFB_MMP_Events.GetLayerInfo, "LayerInfo1");
        MTKFB_MMP_Events.LayerInfo[2] = MMProfileRegisterEvent(MTKFB_MMP_Events.GetLayerInfo, "LayerInfo2");
        MTKFB_MMP_Events.LayerInfo[3] = MMProfileRegisterEvent(MTKFB_MMP_Events.GetLayerInfo, "LayerInfo3");
        MTKFB_MMP_Events.IOCtrl = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "IOCtrl");
        MTKFB_MMP_Events.Debug = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "Debug");
		MTKFB_MMP_Events.primary_seq_config = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "primary_seq_config");
		MTKFB_MMP_Events.primary_seq_trigger=  MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "primary_seq_trigger");
		MTKFB_MMP_Events.primary_seq_wdma0_efo=  MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "primary_seq_wdma0_efo");
		MTKFB_MMP_Events.primary_seq_rdma0_sof=  MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "primary_seq_rdma0_sof");
		MTKFB_MMP_Events.primary_seq_rdma0_eof=  MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "primary_seq_rdma0_eof");
		MTKFB_MMP_Events.external_seq_config = MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "external_seq_config");
		MTKFB_MMP_Events.external_seq_trigger=	MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "external_seq_trigger");
		MTKFB_MMP_Events.external_seq_wdma0_efo=  MMProfileRegisterEvent(MTKFB_MMP_Events.MTKFB, "external_seq_wdma0_efo");
        MMProfileEnableEventRecursive(MTKFB_MMP_Events.MTKFB, 1);
    }
}

#include <linux/ftrace_event.h>

static met_log_map dprec_met_info[DISP_SESSION_MEMORY+2] =
{
    {"DSI-LCM",     0,  0},
    {"OVL0-DSI",    0,  0},
    {"OVL0-MHL",      0,  0},
    {"OVL0-SMS",      0,  0},
};

#define absabs(a) (((a) < 0) ? -(a) : (a))
#define DISP_INTERNAL_BUFFER_COUNT 3
#define FRM_UPDATE_SEQ_CACHE_NUM (DISP_INTERNAL_BUFFER_COUNT+1)
static disp_frm_seq_info frm_update_sequence[FRM_UPDATE_SEQ_CACHE_NUM];
static disp_frm_seq_info frm_update_sequence_mem[FRM_UPDATE_SEQ_CACHE_NUM];
static unsigned int frm_update_cnt;
static unsigned int frm_update_cnt_mem;

static unsigned long __read_mostly tracing_mark_write_addr = 0;
static void inline __mt_update_tracing_mark_write_addr(void)
{
    	if(unlikely(0 == tracing_mark_write_addr))
        	tracing_mark_write_addr = kallsyms_lookup_name("tracing_mark_write");
}

static void dprec_logger_frame_seq_begin(DISP_SESSION_TYPE session_type, unsigned frm_sequence)
{  
    if(frm_sequence <= 0 || session_type <0)
        return ;
	
    if(session_type > DISP_SESSION_MEMORY)
    {
        printk("seq_begin session_type(0x%x) error, seq(%d) \n",session_type, frm_sequence);
        return;
    }
   
    if(dprec_met_info[session_type].begin_frm_seq != frm_sequence)
	{
	    __mt_update_tracing_mark_write_addr();
        event_trace_printk(tracing_mark_write_addr, "S|%d|%s|%d\n", current->tgid, dprec_met_info[session_type].log_name, frm_sequence);
	    dprec_met_info[session_type].begin_frm_seq = frm_sequence;
    }
}

static void dprec_logger_frame_seq_end(DISP_SESSION_TYPE session_type, unsigned frm_sequence)
{
    if(frm_sequence <= 0 || session_type < 0)
        return ;

    if(session_type > DISP_SESSION_MEMORY)
    {
        printk("seq_end session_type(0x%x) , seq(%d) \n",session_type, frm_sequence);
        return;
    }
    
    if(dprec_met_info[session_type].end_frm_seq != frm_sequence)
	{
    	__mt_update_tracing_mark_write_addr();
        event_trace_printk(tracing_mark_write_addr, "F|%d|%s|%d\n", current->tgid, dprec_met_info[session_type].log_name, frm_sequence); 
    	dprec_met_info[session_type].end_frm_seq = frm_sequence;
	}
}

static char* get_frame_seq_state_string(DISP_FRM_SEQ_STATE state)
{
	switch(state)
	{
		case FRM_CONFIG:
			return "FRM_CONFIG";
		case FRM_TRIGGER:
			return "FRM_TRIGGER";
		case FRM_WDMA0_EOF:
			return "FRM_WDMA0_EOF";
		case FRM_RDMA0_SOF:
			return "FRM_RDMA0_SOF";
		case FRM_RDMA0_EOF:
			return "FRM_RDMA0_EOF";
		default:
			return "UNKNOW Frame State";
	}

	return "";
}

void update_frm_seq_info(unsigned int session_id, unsigned int addr, unsigned int addr_offset,unsigned int seq, DISP_FRM_SEQ_STATE state)
{
    int i= 0;
    unsigned device_type = DISP_SESSION_TYPE(session_id);
    //printk("update_frm_seq_info, 0x%x/0x%x/0x%x/%d/%s/%d\n", session_id, addr, addr_offset, seq, get_frame_seq_state_string(state), frm_update_cnt);
	
    if(seq < 0 || session_id < 0 || state > FRM_RDMA0_EOF)
        return ;
    if(device_type > DISP_SESSION_MEMORY)
    {
        printk("seq_end session_id(0x%x) , seq(%d) \n",session_id, seq);
        return;
    }   
	
	if(device_type == DISP_SESSION_PRIMARY || device_type == DISP_SESSION_PRIMARY - 1)
	{
		if(FRM_CONFIG == state)
		{
			frm_update_sequence[frm_update_cnt].state = state;
			frm_update_sequence[frm_update_cnt].session_type = device_type;
			frm_update_sequence[frm_update_cnt].mva= addr;
			frm_update_sequence[frm_update_cnt].max_offset= addr_offset;
			if(seq > 0)
				frm_update_sequence[frm_update_cnt].seq= seq;
			MMProfileLogEx(MTKFB_MMP_Events.primary_seq_config, MMProfileFlagPulse, addr, seq);
		}
		else if(FRM_TRIGGER == state)
		{
			frm_update_sequence[frm_update_cnt].state = FRM_TRIGGER;
			MMProfileLogEx(MTKFB_MMP_Events.primary_seq_trigger, MMProfileFlagPulse, addr, seq);
			dprec_logger_frame_seq_begin(device_type,  frm_update_sequence[frm_update_cnt].seq);
	
			frm_update_cnt++;
			frm_update_cnt%=FRM_UPDATE_SEQ_CACHE_NUM;
		}
		else if(FRM_WDMA0_EOF == state)
		{
			for(i= 0; i< FRM_UPDATE_SEQ_CACHE_NUM; i++)
			{			 
	//			 if((absabs(addr -frm_update_sequence[i].mva) <=  frm_update_sequence[i].max_offset) && (frm_update_sequence[i].state == FRM_TRIGGER))
				if(frm_update_sequence[i].state == FRM_TRIGGER)
				{
					frm_update_sequence[i].state = FRM_WDMA0_EOF;
					frm_update_sequence[i].mva = addr;
					MMProfileLogEx(MTKFB_MMP_Events.primary_seq_wdma0_efo, MMProfileFlagPulse, frm_update_sequence[i].mva, frm_update_sequence[i].seq);
					///break;
				}
			}
		}
		else if(FRM_RDMA0_SOF == state)
		{
			for(i= 0; i< FRM_UPDATE_SEQ_CACHE_NUM; i++)
			{
				if(FRM_WDMA0_EOF == frm_update_sequence[i].state && device_type == DISP_SESSION_PRIMARY && frm_update_sequence[i].mva == addr)
				{
					frm_update_sequence[i].state = FRM_RDMA0_SOF;
					dprec_logger_frame_seq_end(device_type,	frm_update_sequence[i].seq);
					dprec_logger_frame_seq_begin(DISP_SESSION_PRIMARY-1,  frm_update_sequence[i].seq);
					MMProfileLogEx(MTKFB_MMP_Events.primary_seq_rdma0_sof, MMProfileFlagPulse, frm_update_sequence[i].mva, frm_update_sequence[i].seq); 
				}
			}
		} 
		else if(FRM_RDMA0_EOF == state)
		{
			for(i= 0; i< FRM_UPDATE_SEQ_CACHE_NUM; i++)
			{
				if(FRM_RDMA0_SOF == frm_update_sequence[i].state && device_type == DISP_SESSION_PRIMARY && frm_update_sequence[i].mva == addr)
				{
					frm_update_sequence[i].state = FRM_RDMA0_EOF;
					dprec_logger_frame_seq_end(DISP_SESSION_PRIMARY-1,	frm_update_sequence[i].seq );
					MMProfileLogEx(MTKFB_MMP_Events.primary_seq_rdma0_eof, MMProfileFlagPulse, frm_update_sequence[i].mva, frm_update_sequence[i].seq);
					
				}
			}
		}
	}
	else if(device_type == DISP_SESSION_MEMORY)
	{
		if(FRM_CONFIG == state)
		{
			frm_update_sequence_mem[frm_update_cnt_mem].state = state;
			frm_update_sequence_mem[frm_update_cnt_mem].session_type = device_type;
			frm_update_sequence_mem[frm_update_cnt_mem].mva= addr;
			frm_update_sequence_mem[frm_update_cnt_mem].max_offset= addr_offset;
			if(seq > 0)
				frm_update_sequence_mem[frm_update_cnt_mem].seq= seq;
	
			MMProfileLogEx(MTKFB_MMP_Events.external_seq_config, MMProfileFlagPulse, addr, seq);
		}
		else if(FRM_TRIGGER == state)
		{
			frm_update_sequence_mem[frm_update_cnt_mem].state = FRM_TRIGGER;
			MMProfileLogEx(MTKFB_MMP_Events.external_seq_trigger, MMProfileFlagPulse, addr, seq);
			dprec_logger_frame_seq_begin(device_type,  frm_update_sequence_mem[frm_update_cnt_mem].seq);
	
			frm_update_cnt_mem++;
			frm_update_cnt_mem%=FRM_UPDATE_SEQ_CACHE_NUM;
		}
		else if(FRM_WDMA0_EOF == state)
		{
			for(i= 0; i< FRM_UPDATE_SEQ_CACHE_NUM; i++)
			{			 
	//			  if((absabs(addr -frm_update_sequence_mem[i].mva) <=  frm_update_sequence_mem[i].max_offset) && (frm_update_sequence_mem[i].state == FRM_TRIGGER))
				if(frm_update_sequence_mem[i].state == FRM_TRIGGER)
				{
					MMProfileLogEx(MTKFB_MMP_Events.external_seq_wdma0_efo, MMProfileFlagPulse, frm_update_sequence_mem[i].mva, frm_update_sequence_mem[i].seq);
	
					frm_update_sequence_mem[i].state = FRM_WDMA0_EOF;
					frm_update_sequence_mem[i].mva = addr;
					dprec_logger_frame_seq_end(device_type,	frm_update_sequence_mem[i].seq);
					///break;
				}
			}
		}
	}
}
static __inline int is_layer_enable(unsigned int roi_ctl, unsigned int layer)
{
    return (roi_ctl >> (31 - layer)) & 0x1;
}

static void dump_layer_info(void)
{
	unsigned int i;
	for(i=0;i<4;i++){
		printk("LayerInfo in LCD driver, layer=%d,layer_en=%d, source=%d, fmt=%d, addr=0x%lx, x=%d, y=%d \n\
		w=%d, h=%d, pitch=%d, keyEn=%d, key=%d, aen=%d, alpha=%d \n ", 
	    cached_layer_config[i].layer,   // layer
	    cached_layer_config[i].layer_en,
	    cached_layer_config[i].source,   // data source (0=memory)
	    cached_layer_config[i].fmt, 
	    cached_layer_config[i].addr, // addr 
	    cached_layer_config[i].dst_x,  // x
	    cached_layer_config[i].dst_y,  // y
	    cached_layer_config[i].dst_w, // width
	    cached_layer_config[i].dst_h, // height
	    cached_layer_config[i].src_pitch, //pitch, pixel number
	    cached_layer_config[i].keyEn,  //color key
	    cached_layer_config[i].key,  //color key
	    cached_layer_config[i].aen, // alpha enable
	    cached_layer_config[i].alpha);	
	}
}


// ---------------------------------------------------------------------------
//  FPS Log
// ---------------------------------------------------------------------------

typedef struct {
    long int current_lcd_time_us;
    long int current_te_delay_time_us;
    long int total_lcd_time_us;
    long int total_te_delay_time_us;
    long int start_time_us;
    long int trigger_lcd_time_us;
    unsigned int trigger_lcd_count;

    long int current_hdmi_time_us;
    long int total_hdmi_time_us;
    long int hdmi_start_time_us;
    long int trigger_hdmi_time_us;
    unsigned int trigger_hdmi_count;
} FPS_LOGGER;

static FPS_LOGGER fps = {0};
static FPS_LOGGER hdmi_fps = {0};

static long int get_current_time_us(void)
{
    struct timeval t;
    do_gettimeofday(&t);
    return (t.tv_sec & 0xFFF) * 1000000 + t.tv_usec;
}


static void __inline reset_fps_logger(void)
{
    memset(&fps, 0, sizeof(fps));
}

static void __inline reset_hdmi_fps_logger(void)
{
    memset(&hdmi_fps, 0, sizeof(hdmi_fps));
}

void DBG_OnTriggerLcd(void)
{
    if (!dbg_opt.en_fps_log && !dbg_opt.en_touch_latency_log) return;
    
    fps.trigger_lcd_time_us = get_current_time_us();
    if (fps.trigger_lcd_count == 0) {
        fps.start_time_us = fps.trigger_lcd_time_us;
    }
}

void DBG_OnTriggerHDMI(void)
{
    if (!dbg_opt.en_fps_log && !dbg_opt.en_touch_latency_log) return;
    
    hdmi_fps.trigger_hdmi_time_us = get_current_time_us();
    if (hdmi_fps.trigger_hdmi_count == 0) {
        hdmi_fps.hdmi_start_time_us = hdmi_fps.trigger_hdmi_time_us;
    }
}

void DBG_OnTeDelayDone(void)
{
    long int time;
    
    if (!dbg_opt.en_fps_log && !dbg_opt.en_touch_latency_log) return;

    time = get_current_time_us();
    fps.current_te_delay_time_us = (time - fps.trigger_lcd_time_us);
    fps.total_te_delay_time_us += fps.current_te_delay_time_us;
}


void DBG_OnLcdDone(void)
{
    long int time;

    if (!dbg_opt.en_fps_log && !dbg_opt.en_touch_latency_log) return;

    // deal with touch latency log

    time = get_current_time_us();
    fps.current_lcd_time_us = (time - fps.trigger_lcd_time_us);

#if 0   // FIXME
    if (dbg_opt.en_touch_latency_log && tpd_start_profiling) {

        pr_info("DISP/DBG " "Touch Latency: %ld ms\n", 
               (time - tpd_last_down_time) / 1000);

        pr_info("DISP/DBG " "LCD update time %ld ms (TE delay %ld ms + LCD %ld ms)\n",
               fps.current_lcd_time_us / 1000,
               fps.current_te_delay_time_us / 1000,
               (fps.current_lcd_time_us - fps.current_te_delay_time_us) / 1000);
        
        tpd_start_profiling = 0;
    }
#endif

    if (!dbg_opt.en_fps_log) return;

    // deal with fps log
        
    fps.total_lcd_time_us += fps.current_lcd_time_us;
    ++ fps.trigger_lcd_count;

    if (fps.trigger_lcd_count >= dbg_opt.log_fps_wnd_size) {

        long int f = fps.trigger_lcd_count * 100 * 1000 * 1000 
                     / (time - fps.start_time_us);

        long int update = fps.total_lcd_time_us * 100 
                          / (1000 * fps.trigger_lcd_count);

        long int te = fps.total_te_delay_time_us * 100 
                      / (1000 * fps.trigger_lcd_count);

        long int lcd = (fps.total_lcd_time_us - fps.total_te_delay_time_us) * 100
                       / (1000 * fps.trigger_lcd_count);

        pr_info("DISP/DBG " "MTKFB FPS: %ld.%02ld, Avg. update time: %ld.%02ld ms "
               "(TE delay %ld.%02ld ms, LCD %ld.%02ld ms)\n",
               f / 100, f % 100,
               update / 100, update % 100,
               te / 100, te % 100,
               lcd / 100, lcd % 100);
		reset_fps_logger();
	}
}

void DBG_OnHDMIDone(void)
{
    long int time;

    if (!dbg_opt.en_fps_log && !dbg_opt.en_touch_latency_log) return;

    // deal with touch latency log

    time = get_current_time_us();
    hdmi_fps.current_hdmi_time_us = (time - hdmi_fps.trigger_hdmi_time_us);


    if (!dbg_opt.en_fps_log) return;

    // deal with fps log
        
    hdmi_fps.total_hdmi_time_us += hdmi_fps.current_hdmi_time_us;
    ++ hdmi_fps.trigger_hdmi_count;

    if (hdmi_fps.trigger_hdmi_count >= dbg_opt.log_fps_wnd_size) {

        long int f = hdmi_fps.trigger_hdmi_count * 100 * 1000 * 1000 
                     / (time - hdmi_fps.hdmi_start_time_us);

        long int update = hdmi_fps.total_hdmi_time_us * 100 
                          / (1000 * hdmi_fps.trigger_hdmi_count);

        pr_info("DISP/DBG " "[HDMI] FPS: %ld.%02ld, Avg. update time: %ld.%02ld ms\n",
               f / 100, f % 100,
               update / 100, update % 100);
        
        reset_hdmi_fps_logger();
    }
}

// ---------------------------------------------------------------------------
//  Command Processor
// ---------------------------------------------------------------------------
extern void mtkfb_clear_lcm(void);
extern void hdmi_force_init(void);
extern int DSI_BIST_Pattern_Test(DISP_MODULE_ENUM module, cmdqRecHandle cmdq, bool enable, unsigned int color);

bool get_ovl1_to_mem_on()
{
    return enable_ovl1_to_mem;
}

void switch_ovl1_to_mem(bool on)
{
    enable_ovl1_to_mem = on;
    pr_info("DISP/DBG " "switch_ovl1_to_mem %d\n", enable_ovl1_to_mem);
}
static int _draw_line(unsigned long addr, int l, int t, int r, int b, int linepitch, unsigned int color)
{
	int i = 0;

	if(l>r || b<t) return -1;
	
	if(l == r)
	{ // vertical line
		for(i = 0;i<(b-t);i++)
		{
			*(unsigned long*)(addr+(t+i)*linepitch+l*4) = color;
		}
	}
	else if(t == b)
	{ // horizontal line
		for(i = 0;i<(r-l);i++)
		{
			*(unsigned long*)(addr+t*linepitch+(l+i)*4) = color;
		}
	}
	else
	{ // tile line, not support now
		return -1;
	}

	return 0;
}

static int _draw_rect(unsigned int addr, int l, int t, int r, int b, unsigned int linepitch, unsigned int color)
{
	int ret = 0;
	ret += _draw_line(addr, l, t, r, t, linepitch, color);
	ret += _draw_line(addr, l, t, l, b, linepitch, color);
	ret += _draw_line(addr, r, t, r, b, linepitch, color);
	ret += _draw_line(addr, l, b, r, b, linepitch, color);
	return ret;
}

static void _draw_block(unsigned long addr, unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int linepitch, unsigned int color)
{
	int i = 0;
	int j = 0;
	unsigned long start_addr = addr+linepitch*y+x*4;
	DISPMSG("addr=0x%lx, start_addr=0x%lx, x=%d,y=%d,w=%d,h=%d,linepitch=%d, color=0x%08x\n", addr, start_addr, x,y,w,h,linepitch,color);
	for(j=0;j<h;j++)
	{
		for(i = 0;i<w;i++)
		{
			*(unsigned long*)(start_addr + i*4 + j*linepitch) = color;
		}
	}
}
extern void  smp_inner_dcache_flush_all(void);

static int g_display_debug_pattern_index = 0;
void _debug_pattern(unsigned long mva, unsigned long va, unsigned int w, unsigned int h, unsigned int linepitch, unsigned int color, unsigned int layerid, unsigned int bufidx)
{
	if(g_display_debug_pattern_index == 0)
		return;
	
	unsigned int addr = 0;
	unsigned int layer_size = 0;
	unsigned int mapped_size = 0;
	
	unsigned int bcolor = 0xff808080;
	if(layerid == 0) bcolor = 0x0000ffff;
	else if(layerid == 1) bcolor = 0x00ff00ff;
	else if(layerid == 2) bcolor = 0xff0000ff;
	else if(layerid == 3) bcolor = 0xffff00ff;

	if(va)
	{
		addr = va;
	}
	else
	{
		layer_size = linepitch*h;
		m4u_mva_map_kernel(mva, layer_size,&addr, &mapped_size);		
		if(mapped_size == 0)
		{
			DISPERR("m4u_mva_map_kernel failed\n");
			return;
		}
	}
	
	switch(g_display_debug_pattern_index)
	{
		case 1:
		{
			unsigned int resize_factor = layerid+1;

			_draw_rect(addr, w/10*resize_factor+0, h/10*resize_factor+0, w/10*(10-resize_factor)-0, h/10*(10-resize_factor)-0, linepitch, bcolor);
			_draw_rect(addr, w/10*resize_factor+1, h/10*resize_factor+1, w/10*(10-resize_factor)-1, h/10*(10-resize_factor)-1, linepitch, bcolor);
			_draw_rect(addr, w/10*resize_factor+2, h/10*resize_factor+2, w/10*(10-resize_factor)-2, h/10*(10-resize_factor)-2, linepitch, bcolor);
			_draw_rect(addr, w/10*resize_factor+3, h/10*resize_factor+3, w/10*(10-resize_factor)-3, h/10*(10-resize_factor)-3, linepitch, bcolor);
			break;
		}
		case 2:
		{
			int x = 0;
			int y = 0;
			int bw = 20;
			int bh = 20;
			_draw_block(addr, bufidx%(w/bw)*bw, bufidx%(w*h/bh/bh)/(w/bh)*bh, bw, bh, linepitch, bcolor);
			break;
		}
		case 3:
		{
			int x = 0;
			int y = 0;
			int bw = 20;
			int bh = 20;
			_draw_block(addr, bufidx%(w/bw)*bw, bufidx%(w*h/bh/bh)/(w/bh)*bh, bw, bh, linepitch, bcolor);
			break;
		}
	}
	
//	smp_inner_dcache_flush_all();
//	outer_flush_all();
	if(mapped_size)
	{
		m4u_mva_unmap_kernel(addr, layer_size, addr);
	}

	return;
}

#define DEBUG_FPS_METER_SHOW_COUNT	60
static int _fps_meter_array[DEBUG_FPS_METER_SHOW_COUNT]={0};
static unsigned long long _last_ts = 0;

void _debug_fps_meter(unsigned long mva, unsigned long va, unsigned int w, unsigned int h, unsigned int linepitch, unsigned int color, unsigned int layerid, unsigned int bufidx)
{
	int i = 0;
	unsigned long addr = 0;
	unsigned int layer_size = 0;
	unsigned int mapped_size = 0;
	unsigned long long current_ts = sched_clock();
	unsigned long long t = current_ts;
	unsigned long mod = 0;
	unsigned long current_idx = 0;
	unsigned long long l = _last_ts;
	
	if(g_display_debug_pattern_index != 3)
		return;
	DISPMSG("layerid=%d\n", layerid);
	_last_ts = current_ts;

	if(va)
	{
		addr = va;
	}
	else
	{
		layer_size = linepitch*h;
		m4u_mva_map_kernel(mva, layer_size,&addr, &mapped_size);		
		if(mapped_size == 0)
		{
			DISPERR("m4u_mva_map_kernel failed\n");
			return;
		}
	}

	mod = do_div(t, 1000*1000*1000);
	do_div(l, 1000*1000*1000);
	if(t!=l)
	{
		memset((void*)_fps_meter_array, 0, sizeof(_fps_meter_array));
		_draw_block(addr, 0, 10, w, 36, linepitch, 0x00000000);
	}

	current_idx = mod/1000/16666;
	DISPMSG("mod=%ld, current_idx=%ld\n", mod, current_idx);
	_fps_meter_array[current_idx] ++;
	for(i=0;i<DEBUG_FPS_METER_SHOW_COUNT;i++)
	{
		if(_fps_meter_array[i])
		{
			_draw_block(addr, i*18, 10, 18, 18*_fps_meter_array[i], linepitch, 0xff0000ff);
		}
		else
		{
			//_draw_block(addr, i*18, 10, 18, 18, linepitch, 0x00000000);
		}
	}
	
//	smp_inner_dcache_flush_all();
//	outer_flush_all();
	if(mapped_size)
	{
		m4u_mva_unmap_kernel(addr, layer_size, addr);
	}

	return;
}

extern int DAL_Clean(void);
extern int DAL_Printf(const char *fmt, ...);
extern void DSI_ChangeClk(DISP_MODULE_ENUM module,UINT32 clk);
extern int dpmgr_module_notify(DISP_MODULE_ENUM module, DISP_PATH_EVENT event);

static void process_dbg_opt(const char *opt)
{
	if (0 == strncmp(opt, "dsipattern", 10))
	{
		char *p = (char *)opt + 11;
		unsigned int pattern = (unsigned int) simple_strtoul(p, &p, 16);

		if (pattern) 
		{
			DSI_BIST_Pattern_Test(DISP_MODULE_DSI0,NULL,true,pattern);
			DISPMSG("enable dsi pattern: 0x%08x\n", pattern);
		}
		else 
		{
			primary_display_manual_lock();
			DSI_BIST_Pattern_Test(DISP_MODULE_DSI0,NULL,false,0);
			primary_display_manual_unlock();
			return;
		}
	}
	else if (0 == strncmp(opt, "trigger", 7))
	{
		int i = 0;
		for(i=0;i<1200;i++)
		{
			dpmgr_module_notify(DISP_MODULE_AAL, DISP_PATH_EVENT_TRIGGER);
		}
	}
	else if (0 == strncmp(opt, "diagnose", 8))
	{
		primary_display_diagnose();
		return;
	}
	else if (0 == strncmp(opt, "_efuse_test", 11))
	{
		primary_display_check_test();
	}
	else if (0 == strncmp(opt, "trigger", 7))
	{
		display_primary_path_context *ctx = primary_display_path_lock("debug");
		if(ctx)
		{
			dpmgr_signal_event(ctx->dpmgr_handle, DISP_PATH_EVENT_TRIGGER);
		}
		primary_display_path_unlock("debug");
		
		return;
	}
	else if (0 == strncmp(opt, "dprec_reset", 11))
	{
		dprec_logger_reset_all();
		return;
	}
	else if (0 == strncmp(opt, "suspend", 4))
    {
    	primary_display_suspend();
		return;
    }
else if (0 == strncmp(opt, "ata",3))
    {
    	mtkfb_fm_auto_test();
		return;
    }
    else if (0 == strncmp(opt, "resume", 4))
    {
    		primary_display_resume();
    }
    else if (0 == strncmp(opt, "dalprintf", 9))
    {
    		DAL_Printf("display aee layer test\n");
    }
    else if (0 == strncmp(opt, "dalclean", 8))
    {
    		DAL_Clean();
    }
	else if (0 == strncmp(opt, "DP", 2))
	{
		char *p = (char *)opt + 3;
		unsigned int pattern = (unsigned int) simple_strtoul(p, &p, 16);
		g_display_debug_pattern_index = pattern;
		return;
	}
	else if(0==strncmp(opt,"dsi0_clk:",9))
   	{
        char*p=(char*)opt+9;
        UINT32 clk=simple_strtoul(p, &p, 10);
        DSI_ChangeClk(DISP_MODULE_DSI0,clk);
    }
    else if (0 == strncmp(opt, "diagnose", 8))
    {
    	primary_display_diagnose();
		return;
    }
		else if (0 == strncmp(opt, "switch:", 7))
    	{
        	char*p=(char*)opt+7;
        	UINT32 mode=simple_strtoul(p, &p, 10);
    		primary_display_switch_dst_mode(mode%2);
			return;
    	}
		else if (0 == strncmp(opt, "disp_mode:", 10))
    	{
        	char*p=(char*)opt+10;
        	gTriggerDispMode=simple_strtoul(p, &p, 10);
        	DISPMSG("DDP: gTriggerDispMode=%d\n", gTriggerDispMode);    
    	}
    else if (0 == strncmp(opt, "cmmva_dprec", 11))
    {
		dprec_handle_option(0x7);
	}
    else if (0 == strncmp(opt, "cmmpa_dprec", 11))
    {
		dprec_handle_option(0x3);
	}
    else if (0 == strncmp(opt, "dprec", 5))
    {
		char *p = (char *)opt + 6;
		unsigned int option = (unsigned int) simple_strtoul(p, &p, 16);
		dprec_handle_option(option);
	}
    else if (0 == strncmp(opt, "cmdq", 4))
    {
		char *p = (char *)opt + 5;
		unsigned int option = (unsigned int) simple_strtoul(p, &p, 16);
		if(option)
			primary_display_switch_cmdq_cpu(CMDQ_ENABLE);
		else
			primary_display_switch_cmdq_cpu(CMDQ_DISABLE);
	}
    else if (0 == strncmp(opt, "maxlayer", 8))
    {
		char *p = (char *)opt + 9;
		unsigned int maxlayer = (unsigned int) simple_strtoul(p, &p, 10);
		if(maxlayer)
			primary_display_set_max_layer(maxlayer);
		else
			DISPERR("can't set max layer to 0\n");
	}
    else if (0 == strncmp(opt, "primary_reset", 13))
    {
		primary_display_reset();
	}
	else if(0 == strncmp(opt, "esd_check", 9))
	{
		char *p = (char *)opt + 10;
		unsigned int enable = (unsigned int) simple_strtoul(p, &p, 10);
		primary_display_esd_check_enable(enable);
	}
	else if(0 ==strncmp(opt, "cmd:", 4))
	{
		char *p = (char *)opt + 4;
		unsigned int value = (unsigned int) simple_strtoul(p, &p, 5);
		int lcm_cmd[5];
		unsigned int cmd_num=1;
		lcm_cmd[0]=value;
		primary_display_set_cmd(lcm_cmd,cmd_num);
	}
	else if(0 == strncmp(opt, "esd_recovery", 12))
	{
		primary_display_esd_recovery();
	}
	else if(0 == strncmp(opt, "lcm0_reset", 10))
	{
	#if 1
		DISP_CPU_REG_SET(DDP_REG_BASE_MMSYS_CONFIG+0x150,1);
		msleep(10);
		DISP_CPU_REG_SET(DDP_REG_BASE_MMSYS_CONFIG+0x150,0);
		msleep(10);
		DISP_CPU_REG_SET(DDP_REG_BASE_MMSYS_CONFIG+0x150,1);

	#else
#if 0
		mt_set_gpio_mode(GPIO106|0x80000000, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO106|0x80000000, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO106|0x80000000, GPIO_OUT_ONE);
		msleep(10);
		mt_set_gpio_out(GPIO106|0x80000000, GPIO_OUT_ZERO);
		msleep(10);
		mt_set_gpio_out(GPIO106|0x80000000, GPIO_OUT_ONE);
#endif
	#endif
	}
	else if(0 == strncmp(opt, "lcm0_reset0", 11))
	{
		DISP_CPU_REG_SET(DDP_REG_BASE_MMSYS_CONFIG+0x150,0);
	}
	else if(0 == strncmp(opt, "lcm0_reset1", 11))
	{
		DISP_CPU_REG_SET(DDP_REG_BASE_MMSYS_CONFIG+0x150,1);
	}
	else if (0 == strncmp(opt, "cg", 2))
    {
		char *p = (char *)opt + 2;
		unsigned int enable = (unsigned int) simple_strtoul(p, &p, 10);
		primary_display_enable_path_cg(enable);
	}
	else if (0 == strncmp(opt, "ovl2mem:", 8))
    {
        if (0 == strncmp(opt + 8, "on", 2))
            switch_ovl1_to_mem(true);
        else
            switch_ovl1_to_mem(false);
    }
	else if (0 == strncmp(opt, "dump_layer:", 11))
    {
        if (0 == strncmp(opt + 11, "on", 2))
        {           
            char *p = (char *)opt + 14;
            gCapturePriLayerDownX = simple_strtoul(p, &p, 10);
            gCapturePriLayerDownY = simple_strtoul(p+1, &p, 10);
			gCapturePriLayerNum= simple_strtoul(p+1, &p, 10);
			gCapturePriLayerEnable = 1;
			if(gCapturePriLayerDownX==0)
				gCapturePriLayerDownX = 20;
			if(gCapturePriLayerDownY==0)
				gCapturePriLayerDownY = 20;
            printk("dump_layer En %d DownX %d DownY %d,Num %d",gCapturePriLayerEnable,gCapturePriLayerDownX,gCapturePriLayerDownY,gCapturePriLayerNum);
            
        }
        else if (0 == strncmp(opt + 11, "off", 3))
        {
            gCapturePriLayerEnable = 0;
			gCapturePriLayerNum = OVL_LAYER_NUM;
			printk("dump_layer En %d\n",gCapturePriLayerEnable);
        }
    }
    else if (0 == strncmp(opt, "dump_decouple:", 14))
    {
        if (0 == strncmp(opt + 14, "on", 2))
        {           
            char *p = (char *)opt + 17;
            gCapturePriLayerDownX = simple_strtoul(p, &p, 10);
            gCapturePriLayerDownY = simple_strtoul(p+1, &p, 10);
			gCapturePriLayerNum= simple_strtoul(p+1, &p, 10);
			gCaptureWdmaLayerEnable = 1;
			if(gCapturePriLayerDownX==0)
				gCapturePriLayerDownX = 20;
			if(gCapturePriLayerDownY==0)
				gCapturePriLayerDownY = 20;
            printk("dump_decouple En %d DownX %d DownY %d,Num %d",gCaptureWdmaLayerEnable,gCapturePriLayerDownX,gCapturePriLayerDownY,gCapturePriLayerNum);
            
        }
        else if (0 == strncmp(opt + 14, "off", 3))
        {
            gCaptureWdmaLayerEnable = 0;
			printk("dump_decouple En %d\n",gCaptureWdmaLayerEnable);
        }
    }
    else if(0 == strncmp(opt, "bkl:", 4))
    {
        char *p = (char *)opt + 4;
        unsigned int level = (unsigned int) simple_strtoul(p, &p, 10);

        printk("process_dbg_opt(), set backlight level = %d\n", level);
        primary_display_setbacklight(level);
    }
#ifdef ROME_TODO
#error
    if (0 == strncmp(opt, "hdmion", 6))
    {
//	hdmi_force_init();
    }
    else if (0 == strncmp(opt, "fps:", 4))
    {
        if (0 == strncmp(opt + 4, "on", 2)) {
            dbg_opt.en_fps_log = 1;
        } else if (0 == strncmp(opt + 4, "off", 3)) {
            dbg_opt.en_fps_log = 0;
        } else {
            goto Error;
        }
        reset_fps_logger();
    }
    else if (0 == strncmp(opt, "tl:", 3))
    {
        if (0 == strncmp(opt + 3, "on", 2)) {
            dbg_opt.en_touch_latency_log = 1;
        } else if (0 == strncmp(opt + 3, "off", 3)) {
            dbg_opt.en_touch_latency_log = 0;
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "black", 5))
    {
	mtkfb_clear_lcm();
    }
    else if (0 == strncmp(opt, "suspend", 4))
    {
        DISP_PanelEnable(FALSE);
        DISP_PowerEnable(FALSE);
    }
    else if (0 == strncmp(opt, "resume", 4))
    {
        DISP_PowerEnable(TRUE);
        DISP_PanelEnable(TRUE);
    }
    else if (0 == strncmp(opt, "lcm:", 4))
    {
        if (0 == strncmp(opt + 4, "on", 2)) {
            DISP_PanelEnable(TRUE);
        } else if (0 == strncmp(opt + 4, "off", 3)) {
            DISP_PanelEnable(FALSE);
        }
		else if (0 == strncmp(opt + 4, "init", 4)) {
			if (NULL != lcm_drv && NULL != lcm_drv->init) {
        		lcm_drv->init();
    		}
        }else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "cabc:", 5))
    {
        if (0 == strncmp(opt + 5, "ui", 2)) {
			mtkfb_set_backlight_mode(1);
        }else if (0 == strncmp(opt + 5, "mov", 3)) {
			mtkfb_set_backlight_mode(3);
        }else if (0 == strncmp(opt + 5, "still", 5)) {
			mtkfb_set_backlight_mode(2);
        }else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "lcd:", 4))
    {
        if (0 == strncmp(opt + 4, "on", 2)) {
            DISP_PowerEnable(TRUE);
        } else if (0 == strncmp(opt + 4, "off", 3)) {
            DISP_PowerEnable(FALSE);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "vsynclog:", 9))
    {
        if (0 == strncmp(opt + 9, "on", 2))
        {
            EnableVSyncLog = 1;
        } else if (0 == strncmp(opt + 9, "off", 3)) 
        {
            EnableVSyncLog = 0;
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "layer", 5))
    {
        dump_layer_info();
    }
    else if (0 == strncmp(opt, "regw:", 5))
    {
        char *p = (char *)opt + 5;
        unsigned long addr = simple_strtoul(p, &p, 16);
        unsigned long val  = simple_strtoul(p + 1, &p, 16);

        if (addr) {
            OUTREG32(addr, val);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "regr:", 5))
    {
        char *p = (char *)opt + 5;
        unsigned int addr = (unsigned int) simple_strtoul(p, &p, 16);

        if (addr) {
            pr_info("DISP/DBG " "Read register 0x%08x: 0x%08x\n", addr, INREG32(addr));
        } else {
            goto Error;
        }
    }
    else if(0 == strncmp(opt, "dither:", 7))
    {
        unsigned lrs, lgs, lbs, dbr, dbg, dbb;
        char *p = (char *)opt + 7;

        lrs = (unsigned int) simple_strtoul(p, &p, 16);
        p++;
        lgs = (unsigned int) simple_strtoul(p, &p, 16);
        p++;
        lbs = (unsigned int) simple_strtoul(p, &p, 16);
        p++;
        dbr = (unsigned int) simple_strtoul(p, &p, 16);
        p++;
        dbg = (unsigned int) simple_strtoul(p, &p, 16);
        p++;
        dbb = (unsigned int) simple_strtoul(p, &p, 16);
        
        pr_info("DISP/DBG " "process_dbg_opt(), %d %d %d %d %d %d\n", lrs, lgs, lbs, dbr, dbg, dbb);
    }
    else if (0 == strncmp(opt, "mtkfblog:", 9))
    {
        if (0 == strncmp(opt + 9, "on", 2)) {
            mtkfb_log_enable(true);
        } else if (0 == strncmp(opt + 9, "off", 3)) {
            mtkfb_log_enable(false);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "displog:", 8))
    {
        if (0 == strncmp(opt + 8, "on", 2)) {
            disp_log_enable(true);
        } else if (0 == strncmp(opt + 8, "off", 3)) {
            disp_log_enable(false);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "mtkfb_vsynclog:", 15))
    {
        if (0 == strncmp(opt + 15, "on", 2)) {
            mtkfb_vsync_log_enable(true);
        } else if (0 == strncmp(opt + 15, "off", 3)) {
            mtkfb_vsync_log_enable(false);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "log:", 4))
    {
        if (0 == strncmp(opt + 4, "on", 2)) {
			mtkfb_log_enable(true);
			disp_log_enable(true);
        } else if (0 == strncmp(opt + 4, "off", 3)) {
            mtkfb_log_enable(false);
			disp_log_enable(false);
        } else {
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "update", 6))
    {
		DISP_UpdateScreen(0, 0, DISP_GetScreenWidth(), DISP_GetScreenHeight());
    }
    else if (0 == strncmp(opt, "pan_disp", 8))
    {
		mtkfb_pan_disp_test();
    }
    else if (0 == strncmp(opt, "sem_cnt", 7))
    {
		mtkfb_show_sem_cnt();
    }
    else if (0 == strncmp(opt, "hang:", 5))
    {
        if (0 == strncmp(opt + 5, "on", 2)) {
            mtkfb_hang_test(true);
        } else if (0 == strncmp(opt + 5, "off", 3)) {
            mtkfb_hang_test(false);
        } else{
            goto Error;
        }
    }
    else if (0 == strncmp(opt, "cpfbonly:", 9))
    {
        if (0 == strncmp(opt + 9, "on", 2))
        {
            mtkfb_capture_fb_only(true);
        }
        else if (0 == strncmp(opt + 9, "off", 3))
        {
            mtkfb_capture_fb_only(false);
        }
    }
    else if (0 == strncmp(opt, "esd:", 4))
    {
        if (0 == strncmp(opt + 4, "on", 2))
        {
            esd_recovery_pause(FALSE);
        }
        else if (0 == strncmp(opt + 4, "off", 3))
        {
            esd_recovery_pause(TRUE);
        }
    }
    else if (0 == strncmp(opt, "HQA:", 4))
    {
        if (0 == strncmp(opt + 4, "NormalToFactory", 15))
        {
            mtkfb_switch_normal_to_factory();
        }
        else if (0 == strncmp(opt + 4, "FactoryToNormal", 15))
        {
            mtkfb_switch_factory_to_normal();
        }
    }
    else if (0 == strncmp(opt, "mmp", 3))
    {
        init_mtkfb_mmp_events();
    }
    else if (0 == strncmp(opt, "dump_ovl:", 9))
    {
        if (0 == strncmp(opt + 9, "on", 2))
        {
            char *p = (char *)opt + 12;
            gCaptureOvlDownX = simple_strtoul(p, &p, 10);
            gCaptureOvlDownY = simple_strtoul(p+1, &p, 10);
            gCaptureOvlThreadEnable = 1;
			wake_up_process(captureovl_task);
        }
        else if (0 == strncmp(opt + 9, "off", 3))
        {
            gCaptureOvlThreadEnable = 0;
        }
    }
    else if (0 == strncmp(opt, "dump_fb:", 8))
    {
        if (0 == strncmp(opt + 8, "on", 2))
        {
            char *p = (char *)opt + 11;
            gCaptureFBDownX = simple_strtoul(p, &p, 10);
            gCaptureFBDownY = simple_strtoul(p+1, &p, 10);
            gCaptureFBPeriod = simple_strtoul(p+1, &p, 10);
            gCaptureFBEnable = 1;
			wake_up_interruptible(&gCaptureFBWQ);
        }
        else if (0 == strncmp(opt + 8, "off", 3))
        {
            gCaptureFBEnable = 0;
        }   
    }
    else
	{
	    if (disphal_process_dbg_opt(opt))
		goto Error;
	}

    return;

Error:
    pr_err("DISP/ERROR " "parse command error!\n\n%s", STR_HELP);
#endif
}


static void process_dbg_cmd(char *cmd)
{
    char *tok;
    
    pr_info("DISP/DBG " "[mtkfb_dbg] %s\n", cmd);
    
    while ((tok = strsep(&cmd, " ")) != NULL)
    {
        process_dbg_opt(tok);
    }
}


// ---------------------------------------------------------------------------
//  Debug FileSystem Routines
// ---------------------------------------------------------------------------

struct dentry *mtkfb_dbgfs = NULL;


static ssize_t debug_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}


static char debug_buffer[4096+DPREC_ERROR_LOG_BUFFER_LENGTH];
extern int mtkfb_fence_get_debug_info(int buf_len, unsigned char *stringbuf);

int debug_get_info(unsigned char *stringbuf, int buf_len)
{
	int i = 0;
	int n = 0;
	
	DISPFUNC();
	
	n += mtkfb_get_debug_state(stringbuf + n, buf_len-n); 	                          DISPMSG("%s,%d, n=%d\n", __func__, __LINE__, n);
	
	n += primary_display_get_debug_state(stringbuf + n, buf_len-n);             DISPMSG("%s,%d, n=%d\n", __func__, __LINE__, n);
	
	n += disp_sync_get_debug_info(stringbuf + n, buf_len-n);                   	DISPMSG("%s,%d, n=%d\n", __func__, __LINE__, n);	
	
	n += dprec_logger_get_result_string_all(stringbuf + n, buf_len-n);	DISPMSG("%s,%d, n=%d\n", __func__, __LINE__, n);	
	
	n += primary_display_check_path(stringbuf + n, buf_len-n);	DISPMSG("%s,%d, n=%d\n", __func__, __LINE__, n);	
	
	n += dprec_logger_get_buf(DPREC_LOGGER_ERROR, stringbuf + n, buf_len-n);	DISPMSG("%s,%d, n=%d\n", __func__, __LINE__, n);
	
	n += dprec_logger_get_buf(DPREC_LOGGER_FENCE, stringbuf + n, buf_len-n);	DISPMSG("%s,%d, n=%d\n", __func__, __LINE__, n);	

	n += dprec_logger_get_buf(DPREC_LOGGER_HWOP, stringbuf + n, buf_len-n); DISPMSG("%s,%d, n=%d\n", __func__, __LINE__, n);	

	stringbuf[n++] = 0;
	return n;
}

void debug_info_dump_to_printk(char *buf, int buf_len)
{
	int i = 0;
	int n = buf_len;
	for(i=0;i<n;i+=256)
		DISPMSG("%s", buf+i);
}

static ssize_t debug_read(struct file *file,
                          char __user *ubuf, size_t count, loff_t *ppos)
{
	const int debug_bufmax = sizeof(debug_buffer) - 1;
	int i = 0;
	int n = 0;
	
	DISPFUNC();
	
	n += debug_get_info(debug_buffer + n, debug_bufmax-n);
	//debug_info_dump_to_printk();
	return simple_read_from_buffer(ubuf, count, ppos, debug_buffer, n);
}

static ssize_t debug_write(struct file *file,
                           const char __user *ubuf, size_t count, loff_t *ppos)
{
    const int debug_bufmax = sizeof(debug_buffer) - 1;
	size_t ret;

	ret = count;

	if (count > debug_bufmax) 
        count = debug_bufmax;

	if (copy_from_user(&debug_buffer, ubuf, count))
		return -EFAULT;

	debug_buffer[count] = 0;

    process_dbg_cmd(debug_buffer);

    return ret;
}


static struct file_operations debug_fops = {
	.read  = debug_read,
    .write = debug_write,
	.open  = debug_open,
};

#ifdef MTKFB_DEBUG_FS_CAPTURE_LAYER_CONTENT_SUPPORT

static ssize_t layer_debug_open(struct inode *inode, struct file *file)
{
    MTKFB_LAYER_DBG_OPTIONS *dbgopt;
    ///record the private data
    file->private_data = inode->i_private;
    dbgopt = (MTKFB_LAYER_DBG_OPTIONS *)file->private_data;

    dbgopt->working_size = DISP_GetScreenWidth()*DISP_GetScreenHeight()*2 + 32;
    dbgopt->working_buf = (unsigned long)vmalloc(dbgopt->working_size);
    if(dbgopt->working_buf == 0)
        pr_info("DISP/DBG Vmalloc to get temp buffer failed\n");

    return 0;
}


static ssize_t layer_debug_read(struct file *file,
                          char __user *ubuf, size_t count, loff_t *ppos)
{
	return 0;
}


static ssize_t layer_debug_write(struct file *file,
                           const char __user *ubuf, size_t count, loff_t *ppos)
{
    MTKFB_LAYER_DBG_OPTIONS *dbgopt = (MTKFB_LAYER_DBG_OPTIONS *)file->private_data;

    pr_info("DISP/DBG " "mtkfb_layer%d write is not implemented yet \n", dbgopt->layer_index);

    return count;
}

static int layer_debug_release(struct inode *inode, struct file *file)
{
    MTKFB_LAYER_DBG_OPTIONS *dbgopt;
    dbgopt = (MTKFB_LAYER_DBG_OPTIONS *)file->private_data;

    if(dbgopt->working_buf != 0)
        vfree((void *)dbgopt->working_buf);

    dbgopt->working_buf = 0;

    return 0;
}


static struct file_operations layer_debug_fops = {
	.read  = layer_debug_read,
    .write = layer_debug_write,
	.open  = layer_debug_open,
    .release = layer_debug_release,
};

#endif

void DBG_Init(void)
{
    mtkfb_dbgfs = debugfs_create_file("mtkfb",
        S_IFREG|S_IRUGO, NULL, (void *)0, &debug_fops);

    memset(&dbg_opt, sizeof(dbg_opt), 0);
	memset(&fps, sizeof(fps), 0);
    dbg_opt.log_fps_wnd_size = DEFAULT_LOG_FPS_WND_SIZE;
	// xuecheng, enable fps log by default
	dbg_opt.en_fps_log = 1;

#ifdef MTKFB_DEBUG_FS_CAPTURE_LAYER_CONTENT_SUPPORT
    {
        unsigned int i;
        unsigned char a[13];

        a[0] = 'm';
        a[1] = 't';
        a[2] = 'k';
        a[3] = 'f';
        a[4] = 'b';
        a[5] = '_';
        a[6] = 'l';
        a[7] = 'a';
        a[8] = 'y';
        a[9] = 'e';
        a[10] = 'r';
        a[11] = '0';
        a[12] = '\0';
        
        for(i=0;i<DDP_OVL_LAYER_MUN;i++)
        {
            a[11] = '0' + i;
            mtkfb_layer_dbg_opt[i].layer_index = i;
            mtkfb_layer_dbgfs[i] = debugfs_create_file(a,
                S_IFREG|S_IRUGO, NULL, (void *)&mtkfb_layer_dbg_opt[i], &layer_debug_fops);
        }
    }
#endif    
}


void DBG_Deinit(void)
{
    debugfs_remove(mtkfb_dbgfs);
#ifdef MTKFB_DEBUG_FS_CAPTURE_LAYER_CONTENT_SUPPORT
    {
        unsigned int i;
        
        for(i=0;i<DDP_OVL_LAYER_MUN;i++)
            debugfs_remove(mtkfb_layer_dbgfs[i]);
    }
#endif
}


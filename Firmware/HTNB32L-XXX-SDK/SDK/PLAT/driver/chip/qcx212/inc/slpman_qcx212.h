/****************************************************************************
 *
 * Copy right:   2017-, Copyrigths of Qualcomm Ltd.
 * File name:    slp_manager.h
 * Description:  QCX212 platform sleep management header file
 * History:      09/28/2018    Originated by wqzhao
 *
 ****************************************************************************/


#ifndef SLP_MANAGER_H
#define SLP_MANAGER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "mem_map.h"

/*
AT+ECPMUCFG=0,0	disable pmu and can debug on keil
AT+ECPMUCFG=1,1	enable pmu and enter wfi only
AT+ECPMUCFG=1,2	enable pmu and can enter sleep1
AT+ECPMUCFG=1,3	enable pmu and can enter sleep2
AT+ECPMUCFG=1,4	enable pmu and can enter hibernate
AT+ECPMUCFG?      check current pmu state
*/

#define LOW_POWER_AT_TEST       1

#define NEW_WAKEUP_METHOD	    0

#define GD_FALSH                1
#define PY_FLASH                0



#define PMU_DEFAULT_BL_TIME                     (11)

#if GD_FALSH
#define PMU_FLASH_WRITE_TIME_2SECTOR            (77+4)             // page write max time 2.4ms, 2 sector 76.8ms, 4ms Vbat ADC check
#define PMU_PREHIB_EXPECTED_CODE_RUNNING_TIME     (PMU_FLASH_WRITE_TIME_2SECTOR+2)			// time for code running before sleep
#define PMU_PRESLP2_EXPECTED_CODE_RUNNING_TIME     (PMU_FLASH_WRITE_TIME_2SECTOR+2)			// time for code running before sleep
#endif


#define PMU_WORTH_HIB_TIME                   (50+PMU_PREHIB_EXPECTED_CODE_RUNNING_TIME)
#define PMU_POSTHIB_EXPECTED_CODE_RUNNING_TIME      (20)

#define PMU_WORTH_SLP2_TIME                   (50+PMU_PRESLP2_EXPECTED_CODE_RUNNING_TIME)
#define PMU_POSTSLP2_EXPECTED_CODE_RUNNING_TIME     (20)

#define PMU_PRESLP_EXPECTED_CODE_RUNNING_TIME     (5)
#define PMU_POSTSLP_EXPECTED_CODE_RUNNING_TIME     (5)
#define PMU_WORTH_SLEEP_TIME                (5+PMU_PRESLP_EXPECTED_CODE_RUNNING_TIME+PMU_POSTSLP_EXPECTED_CODE_RUNNING_TIME)


#define DEEPSLP_TIMER_MAXRANGE			0x7C745A00				// ticks in 580 hours

typedef enum
{
	DEEPSLP_TIMER_ID0 = 0,		// ID0-ID6 0-580 hours
	DEEPSLP_TIMER_ID1,
	DEEPSLP_TIMER_ID2,
	DEEPSLP_TIMER_ID3,
	DEEPSLP_TIMER_ID4,
	DEEPSLP_TIMER_ID5,
	DEEPSLP_TIMER_ID6,
	DEEPSLP_TIMER_ID7 = 7,		// fixed to AONTimer, no longer than 2 hours
	DEEPSLP_TIMER_ID8 = 8,		// !! for Qualcomm internal use only !!
	DEEPSLP_TIMER_ID9 = 9,		// !! for Qualcomm internal use only !!
}slpManTimerID_e;


typedef struct
{
    uint16_t bl_time;
    uint16_t flash_write_time;
    uint16_t prehib_coderun_time;
    uint16_t preslp2_coderun_time;
    uint16_t preslp1_coderun_time;
    uint16_t posthib_coderun_time;
    uint16_t postslp2_coderun_time;
    uint16_t postslp1_coderun_time;
    uint16_t worth_hib_time;
    uint16_t worth_slp2_time;
    uint16_t worth_slp1_time;
}pmuTimeCfg_t;

typedef struct
{
	bool gp0_posedge_en;
	bool gp0_negedge_en;
	bool gp1_posedge_en;
	bool gp1_negedge_en;
	bool gp2_posedge_en;
	bool gp2_negedge_en;
}padWakeupSettings_t;


typedef enum
{
	AonIOLatch_Disable = 0,
	AonIOLatch_Enable = 1,
}IOLatchEn;



//boot flag defination
typedef enum
{
	BOOT_FROM_POWER_ON = 0x0,
	BOOT_FROM_UNDEF = 0x1,
	BOOT_FROM_SLEEP1 = 0x4,
	BOOT_FROM_SLEEP2 = 0x5,
	BOOT_FROM_HIBERNATE1 = 0x6,
	BOOT_FROM_HIBERNATE2 = 0x7,
}BootFlag_e;

/**
  \addtogroup slpman_interface_gr
  \{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define SLP_CALLBACK_USRDEF_MAX_NUM		8
#define SLP_PLAT_VOTE_MAX_NUM	32
#define SLP_PLAT_VOTE_INFO_LEN	9

// Flash mem for APP backup in power management-----just consider based on PS/PHY + offset(8K)
#define APP_BACKUP_XIP_ADDR         (FLASH_XIP_ADDR + 0x3B8000 + 0x2000)			// should fix later
#define APP_BACKUP_NONXIP_ADDR      (APP_BACKUP_XIP_ADDR - FLASH_XIP_ADDR)
#define APP_BACKUP_SIZE             (0x1000)


/**
  \brief slpManLpState, to indicate in which sleep state should we call the backup and restore function.
 */
typedef enum
{
	SLPMAN_INVALID_STATE = 0,       /**< invalid deepsleep state */
	SLPMAN_SLEEP1_STATE,            /**< sleep1 state */
	SLPMAN_SLEEP2_STATE,            /**< sleep2 state */
	SLPMAN_HIBERNATE_STATE,         /**< hibernate state */
    MAX_SLEEP_STATE,
}slpManLpState;

/**
  \brief enum of sleep status for voting. we should always try to vote a higher status for power saving.
 */
typedef enum
{
    SLP_ACTIVE_STATE = 0,           /**< active state */
	SLP_IDLE_STATE,                 /**< idle state */
	SLP_SLP1_STATE,                 /**< sleep1 state */
    SLP_SLP2_STATE,                 /**< sleep2 state */
	SLP_HIB_STATE,                  /**< hibernate state */
	SLP_STATE_MAX
}slpManSlpState_t;


typedef enum
{
    WAKEUP_FROM__POR,
    WAKEUP_FROM_RTC,
    WAKEUP_FROM_PAD,
}slpManWakeSrc_e;

/**
  \brief definition of backup callback(called before sleep)
 */
typedef void(* slpManBackupCb_t)(void *pdata, slpManLpState state);
/**
  \brief definition of restore callback(called after sleep)
 */
typedef void(* slpManRestoreCb_t)(void *pdata, slpManLpState state);
/**
  \brief definition of deepsleep timer callback function
 */
typedef void (*slpManUsrDeepSlpTimerCb_Func)(uint8_t id);

typedef slpManSlpState_t (* slpUserdefSleepCb_Func)(void);

/**
  \brief a stucture for array to store backup function and it's parameters.
 */
typedef struct
{
	slpManBackupCb_t backup_func;       /**< function execute before enter sleep */
	void *pdata;                        /**< a pointer to the input param of callback function */
	slpManLpState state;                /**< define in which state should the backup_func executed */
}slpManBackupCbFunc_t;

/**
  \brief a stucture for array to store restore function and it's parameters.
 */
typedef struct
{
	slpManBackupCb_t restore_func;      /**< function execute after sleep is exited. When sleep2 or hibernate success the function will not execute */
	void *pdata;                        /**< a pointer to the input param of callback function */
	slpManLpState state;                /**< define in which state should the restore_func executed */
}slpManRestoreCbFunc_t;

/**
  \brief enum of error code used in slp manager
 */
typedef enum
{
	RET_TRUE,                           /**< Error code: no error */
	RET_UNKNOW_FALSE,                   /**< Error code: unknow error */
	RET_CB_ARRAY_FULL,                  /**< Error code: callback array is full */
	RET_CALLBACK_EXIST,                 /**< Error code: duplicate callback in callback array */
	RET_CALLBACK_UNEXIST,               /**< Error code: callback unexist */
	RET_VOTE_SLP_OVERFLOW,              /**< Error code: vote overflow (above 256) */
	RET_VOTE_SLP_UNDERFLOW,             /**< Error code: vote underflow (below 0) */
	RET_ZERO_NAME_LEN,                  /**< Error code: input name is empty */
	RET_INVALID_NAME,                   /**< Error code: input name has illegal character */
	RET_VOTE_HANDLE_FULL,               /**< Error code: no place for more vote handle */
	RET_INVALID_HANDLE,                 /**< Error code: vote handle is invalid */
	RET_INVALID_VOTE,                   /**< Error code: the vote state is invalid */
	RET_VOTE_CONFLICT,                  /**< Error code: vote conflict */
	RET_GIVEBACK_FAILED,                /**< Error code: vote handle give back error */
	RET_HANDLE_NOT_FOUND,               /**< Error code: vote handle not found */
}slpManRet_t;

/**
  \brief enum of predefined module for backup and restore functions.
 */
typedef enum
{
    SLP_CALLBACK_GPR_MODULE = 0,        /**< sleep callback of gpr module */
    SLP_CALLBACK_PAD_MODULE,            /**< sleep callback of pad module */
    SLP_CALLBACK_GPIO_MODULE,           /**< sleep callback of gpio module */
    SLP_CALLBACK_USART_MODULE,          /**< sleep callback of usart module */
    SLP_CALLBACK_SPI_MODULE,            /**< sleep callback of spi module */
    SLP_CALLBACK_DMA_MODULE,            /**< sleep callback of dma module */
    SLP_CALLBACK_UNILOG_MODULE,         /**< sleep callback of unilog module */
    SLP_CALLBACK_TIMER_MODULE,          /**< sleep callback of timer module */
    SLP_CALLBACK_I2C_MODULE,            /**< sleep callback of i2c module */
    SLP_CALLBACK_ADC_MODULE,            /**< sleep callback of adc module */
    SLP_CALLBACK_SIM_MODULE,            /**< sleep callback of sim module */
    SLP_CALLBACK_L2C_MODULE,            /**< sleep callback of L2C module */
    SLP_CALLBACK_WDT_MODULE,            /**< sleep callback of WDT module */
    SLP_CALLBACK_ATCMD_MODULE,          /**< sleep callback of ATCMD module */
    SLP_CALLBACK_MAX_NUM,
}slpCbModule_t;



typedef enum
{
	// @ 1.8V level�� make sure LDOIO33_18_SEL Pin is high
	IOVOLT_1_65V = 0,
	IOVOLT_1_70V,
	IOVOLT_1_75V,
	IOVOLT_1_80V,
	IOVOLT_1_85V,
	IOVOLT_1_90V,
	IOVOLT_1_95V,
	IOVOLT_2_00V,

	// @ 2.8V level
	IOVOLT_2_65V = 8,
	IOVOLT_2_70V,
	IOVOLT_2_75V,
	IOVOLT_2_80V,
	IOVOLT_2_85V,
	IOVOLT_2_90V,
	IOVOLT_2_95V,
	IOVOLT_3_00V,

	// @ 3.3V level,  make sure LDOIO33_18_SEL Pin is low
	IOVOLT_3_05V = 16,
	IOVOLT_3_10V,
	IOVOLT_3_15V,
	IOVOLT_3_20V,
	IOVOLT_3_25V,
	IOVOLT_3_30V,
	IOVOLT_3_35V,
	IOVOLT_3_40V,

}IOVoltageSel_t;


/**
  \brief enum of predefined module for backup and restore functions.
 */
typedef enum
{
	SLP_VOTE_USART = 0,                 /**< vote module: Usart */
	SLP_VOTE_I2C,                       /**< vote module: I2C */
	SLP_VOTE_SPI,                       /**< vote module: SPI */


	SLP_VOTE_ADC,                       /**< vote module: ADC */
	SLP_VOTE_DMA,                       /**< vote module: DMA */
	SLP_VOTE_TIMER,                     /**< vote module: Timer */
	SLP_VOTE_MAX_NUM,

}slpDrvVoteModule_t;


/**
  \brief a stucture to store driver vote flag and vote counter.
 */
typedef struct
{
	uint32_t drv_vote_slp_bitmap;            /**< vote bitmap of driver */
	uint8_t vote_counter[SLP_VOTE_MAX_NUM];  /**< vote counter */
}slpManDrvVote_t;


/**
  \brief a stucture to store platform(user defined) vote information.
 */
typedef struct
{
	uint32_t plat_vote_slp1_bitmap;                                                     /**< vote bitmap of sleep1 */
	uint32_t plat_vote_slp2_bitmap;                                                     /**< vote bitmap of sleep2 */
	uint32_t plat_vote_hib_bitmap;                                                      /**< vote bitmap of hibernate */
	uint32_t plat_vote_valid;                                                           /**< valid flag of platform vote */
	uint8_t vote_counter[SLP_PLAT_VOTE_MAX_NUM];                                        /**< platform vote counter */
	uint8_t plat_vote_info[SLP_PLAT_VOTE_MAX_NUM][SLP_PLAT_VOTE_INFO_LEN];              /**< platform vote information */
}slpManPlatVote_t;

/**
  \fn          slpManRet_t slpManRegisterPredefinedBackupCb(slpCbModule_t module, slpManBackupCb_t backup_cb,
                                                             void *pdata, slpManLpState state)
  \brief       Register the predefined backup callback functions.
  \param[in]   module          predefined module
  \param[in]   backup_cb       pointer to the function of backup
  \param[in]   pdata           the input param of backup function
  \param[in]   state           indicate in which state should we excute the callback function.
  \return      error code
*/
slpManRet_t slpManRegisterPredefinedBackupCb(slpCbModule_t module, slpManBackupCb_t backup_cb, void *pdata, slpManLpState state);
/**
  \fn          slpManRet_t slpManRegisterPredefinedRestoreCb(slpCbModule_t module, slpManBackupCb_t restore_cb,
                                                             void *pdata, slpManLpState state)
  \brief       Register the predefined restore callback functions.
  \param[in]   module          predefined module
  \param[in]   restore_cb       pointer to the function of restore
  \param[in]   pdata           the input param of restore function
  \param[in]   state           indicate in which state should we excute the callback function.
  \return      error code
*/
slpManRet_t slpManRegisterPredefinedRestoreCb(slpCbModule_t module, slpManRestoreCb_t restore_cb, void *pdata, slpManLpState state);
/**
  \fn          slpManRet_t slpManRegisterUsrdefinedBackupCb(slpManBackupCb_t backup_cb,
                                                             void *pdata, slpManLpState state)
  \brief       Register the usrdefined backup callback functions.
  \param[in]   backup_cb       pointer to the function of backup
  \param[in]   pdata           the input param of backup function
  \param[in]   state           indicate in which state should we excute the callback function.
  \return      error code
*/
slpManRet_t slpManRegisterUsrdefinedBackupCb(slpManBackupCb_t backup_cb, void *pdata, slpManLpState state);
/**
  \fn          slpManRet_t slpManRegisterUsrdefinedRestoreCb(slpManBackupCb_t restore_cb,
                                                             void *pdata, slpManLpState state)
  \brief       Register the usrdefined restore callback functions.
  \param[in]   restore_cb       pointer to the function of restore
  \param[in]   pdata           the input param of restore function
  \param[in]   state           indicate in which state should we excute the callback function.
  \return      error code
*/
slpManRet_t slpManRegisterUsrdefinedRestoreCb(slpManRestoreCb_t restore_cb, void *pdata, slpManLpState state);
/**
  \fn          slpManRet_t slpManUnregisterPredefinedBackupCb(slpCbModule_t module);
  \brief       Unregister the Predefined callback according to module
  \param[in]   module          predefined module whose callback need unregister
  \return      error code
*/
slpManRet_t slpManUnregisterPredefinedBackupCb(slpCbModule_t module);
/**
  \fn          slpManRet_t slpManUnregisterPredefinedRestoreCb(slpCbModule_t module);
  \brief       Unregister the Predefined callback according to module
  \param[in]   module          predefined module whose callback need unregister
  \return      error code
*/
slpManRet_t slpManUnregisterPredefinedRestoreCb(slpCbModule_t module);
/**
  \fn          slpManRet_t slpManUnregisterUsrdefinedBackupCb(slpManBackupCb_t backup_cb)
  \brief       Unregister the userdefined callback according to the callback function.
  \param[in]   backup_cb          user defined callback which need unregister
  \return      error code
*/
slpManRet_t slpManUnregisterUsrdefinedBackupCb(slpManBackupCb_t backup_cb);
/**
  \fn          slpManRet_t slpManUnregisterUsrdefinedRestoreCb(slpManBackupCb_t restore_cb)
  \brief       Unregister the userdefined callback according to the callback function.
  \param[in]   restore_cb          user defined callback which need unregister
  \return      error code
*/
slpManRet_t slpManUnregisterUsrdefinedRestoreCb(slpManRestoreCb_t restore_cb);
/**
  \fn          void slpManSetDrvVoteMask(uint32_t mask)
  \brief       driver vote mask. The vote of the driver is already done by sdk, but we can mask specific vote result.
  \param[in]   mask  driver vote mask to set
  \return      none
*/
void slpManSetDrvVoteMask(uint32_t mask);
/**
  \fn          void slpManGetDrvBitmap(uint32_t *bitmap, uint32_t *mask)
  \brief       get the driver's vote information
  \param[out]  bitmap        the bitmap of vote information.
  \param[out]  mask          set 1 means the specific driver always alow to sleep
  \return      none.
*/
void slpManGetDrvBitmap(uint32_t *bitmap, uint32_t *mask);

/**
  \fn          slpManSlpState_t slpManPlatGetSlpState(void)
  \brief       get which sleep state can we go now
  \return      the sleep state we can go.
*/
slpManSlpState_t slpManPlatGetSlpState(void);
/**
  \fn          slpManWakeSrc_e slpManGetWakeupSrc(void)
  \brief       get the reason of wakeup, por,rtc,pad...
  \return      wakeup reason.
*/
slpManWakeSrc_e slpManGetWakeupSrc(void);
/**
  \fn          void slpManGetPlatBitmap(uint32_t *slp_bitmap, uint32_t *slp3_bitmap, uint32_t *hib_bitmap)
  \brief       get the bitmap of vote information
  \param[in]   slp_bitmap        the bitmap vote to sleep state.
  \param[in]   slp3_bitmap       the bitmap vote to sleep3 state.
  \param[in]   hib_bitmap        the bitmap vote to hibernate state.
  \return      none.
*/
void slpManGetPlatBitmap(uint32_t *slp_bitmap, uint32_t *slp2_bitmap, uint32_t *hib_bitmap);
/**
  \fn          slpManRet_t slpManApplyPlatVoteHandle(const char* name, uint8_t *handle)
  \brief       apply for a vote handle.
  \param[in]   name        the name of the handle(max length is 8)
  \param[out]  handle      the handle allocate to use
  \return      error code.
*/
slpManRet_t slpManApplyPlatVoteHandle(const char* name, uint8_t *handle);
/**
  \fn          slpManRet_t slpManGivebackPlatVoteHandle(uint8_t handle)
  \brief       give back the platform vote handle
  \param[in]   handle        the platform vote handle.
  \return      error code.
*/
slpManRet_t slpManGivebackPlatVoteHandle(uint8_t handle);
/**
  \fn          uint8_t *slpManGetVoteInfo(uint8_t handle)
  \brief       Get Vote handle Name information according to the handle
  \param[in]   handle        the platform vote handle.
  \return      error code.
*/
uint8_t *slpManGetVoteInfo(uint8_t handle);
/**
  \fn          slpManRet_t slpManPlatVoteDisableSleep(uint8_t handle, slpManSlpState_t status);
  \brief       vote for keep wakeup
  \param[in]   handle        the platform vote handle.
  \param[in]   status        input the sleep status. A handle can only vote to a specific status.
  \return      error code.
*/
slpManRet_t slpManPlatVoteDisableSleep(uint8_t handle, slpManSlpState_t status);
/**
  \fn          slpManRet_t slpManPlatVoteEnableSleep(uint8_t handle, slpManSlpState_t status);
  \brief       vote for going to sleep
  \param[in]   handle        the platform vote handle.
  \param[in]   status        input the sleep status. A handle can only vote to a specific status.
  \return      error code.
*/
slpManRet_t slpManPlatVoteEnableSleep(uint8_t handle, slpManSlpState_t status);
/**
  \fn          slpManRet_t slpManPlatVoteEnableSleep(uint8_t handle, slpManSlpState_t status);
  \brief       vote for going to sleep, and force the specific handle to clear all vote counter. This API should be
  carefully use as it may force sleep without consider the vote counter
  \param[in]   handle        the platform vote handle.
  \param[in]   status        input the sleep status. A handle can only vote to a specific status.
  \return      error code.
*/
slpManRet_t slpManPlatVoteForceEnableSleep(uint8_t handle, slpManSlpState_t status);
/**
  \fn          slpManRet_t slpManCheckVoteState(uint8_t handle, slpManSlpState_t *pstate, uint8_t *counter);
  \brief       Check vote state according to the vote handle
  \param[in]   handle        the platform vote handle.
  \param[out]  pstate        output the sleep status of current handle, if return SLP_ACTIVE_STATE, means this handle is not used.
  \param[out]  counter       vote counter, indicate how many times the specific handle votes,
                slpManPlatVoteDisableSleep let the counter count down,
                slpManPlatVoteEnableSleep let the counter count up
                when conter = 0 the specific handle enable the sleep.
  \return      error code.
*/
slpManRet_t slpManCheckVoteState(uint8_t handle, slpManSlpState_t *pstate, uint8_t *counter);
/**
  \fn          slpManRet_t slpManFindPlatVoteHandle(const char* name, uint8_t *handle);
  \brief       Find a vote handle applied before according to the name
  \param[in]   name        string of handle name.
  \param[out]  handle        the platform vote handle.
  \return      error code.
*/
slpManRet_t slpManFindPlatVoteHandle(const char* name, uint8_t *handle);
/**
  \fn          void slpManSetPmuSleepMode(bool pmu_enable, slpManSlpState_t mode, bool save2flash);
  \brief       Disable or Enable Pmu, and set the deepest sleep mode.
  \param[in]   pmu_enable    when enable pmu set pmu_enable = true
  \param[in]   mode        valid only when pmu_enable is true.
  \param[in]   save2flash        set whether the settings need to flush into flash
  \return      none.
*/
void slpManSetPmuSleepMode(bool pmu_enable, slpManSlpState_t mode, bool save2flash);
/**
  \fn          void slpManRegisterUsrSlpDepthCb(pmuUserdefSleepCb_Func callback);
  \brief       register a user callback function to control sleep depth
  \param[in]   callback    input a callback function
  \return      none.
*/
void slpManRegisterUsrSlpDepthCb(slpUserdefSleepCb_Func callback);
/**
  \fn          void slpManAONIOLatchEn(IOLatchEn en);
  \brief       set to enable io retention during sleep
  \param[in]   en    enable io retention
  \return      none.
*/
void slpManAONIOLatchEn(IOLatchEn en);
/**
  \fn          IOLatchEn slpManAONIOGetLatchCfg(void);
  \brief       get io latch config
  \return      io latch config.
*/
IOLatchEn slpManAONIOGetLatchCfg(void);
/**
  \fn          void slpManAONIOPowerOn(void);
  \brief       power on ldo of aonio. Call this function and than use gpio driver to config the aonio
  \return      none.
*/
void slpManAONIOPowerOn(void);
/**
  \fn          void slpManAONIOPowerOff(void);
  \brief       power off ldo of aonio. All AonIO will switch to low because of power loss
  \return      none.
*/
void slpManAONIOPowerOff(void);

/**
  \fn          slpManSlpState_t slpManGetLastSlpState(void);
  \brief       check last sleep state
  \return      last sleep state
*/
slpManSlpState_t slpManGetLastSlpState(void);
/**
  \fn          void slpManDeepSlpTimerRegisterExpCb(slpManUsrDeepSlpTimerCb_Func cb);
  \brief       register user deep sleep timer expired callback
  \param[in]   cb    input callback function
  \return      none
*/
void slpManDeepSlpTimerRegisterExpCb(slpManUsrDeepSlpTimerCb_Func cb);
/**
  \fn          void slpManDeepSlpTimerStart(uint8_t timerId, uint32_t nMs);
  \brief       Start a User Deep Sleep Timer
  \param[in]   timerId    timer id
  \param[in]   nMs        timer value
  \return      none
*/
void slpManDeepSlpTimerStart(slpManTimerID_e timerId, uint32_t nMs);
/**
  \fn          bool slpManDeepSlpTimerIsRunning(uint8_t timerId);
  \brief       Check whether the deep sleep timer is running
  \param[in]   timerId    timer id
  \return      true or false
*/
bool slpManDeepSlpTimerIsRunning(uint8_t timerId);
/**
  \fn          void slpManDeepSlpTimerDel(uint8_t timerId);
  \brief       Delete a deep sleep timer
  \param[in]   timerId    timer id
  \return      none
*/
void slpManDeepSlpTimerDel(uint8_t timerId);
/**
  \fn          uint32_t slpManDeepSlpTimerRemainMs(uint8_t timerId);
  \brief       Delete a deep sleep timer
  \param[in]   timerId    timer id
  \return      millisecond to timer expired, return 0xffffffff when timerId is invalid
*/
uint32_t slpManDeepSlpTimerRemainMs(uint8_t timerId);
/**
  \fn          void slpManSetWakeupPad(padWakeupSettings_t cfg, uint8_t wakeup_en_pinmap);
  \brief       set wakeup pad
  \param[in]   cfg    wakeup settings
  \return      none
*/
void slpManSetWakeupPad(padWakeupSettings_t cfg);
/**
  \fn          uint8_t slpManGetWakeupPinValue(void);
  \brief       Get the bitmap of 6 Wakeup Pin Value
  \return      a bitmap of 6 pin value
*/
uint8_t slpManGetWakeupPinValue(void);
/**
  \fn          void slpManUpdateUserNVMem(void);
  \brief       Call this API to tell SDK that user NVMem area need update to flash.
               The Update actually happened before sleep2 or hibernate.
  \return      none
*/
void slpManUpdateUserNVMem(void);
/**
  \fn          void slpManRestoreUsrNVMem(void)
  \brief       Restore UserNVMem from FileSystem to ram
  \return      none
*/
void slpManRestoreUsrNVMem(void);
/**
  \fn          uint8_t * slpManGetUsrNVMem(void);
  \brief       Get the pointer to the header of user NVMem which has the maximum size of (2048-32)=2016e.
               User NVMem will be recovered by sdk after wakeup from sleep2, hibernate or power on.
               User can access to user NVMem freely. It's the user's  responsibility to ensure the data integrity of this area.
               Illegal access to the offset larger than 4064 will cause hardfault.
  \return      none
*/
uint8_t * slpManGetUsrNVMem(void);
/**
  \fn          void slpManFlushUsrNVMem(void);
  \brief       Flush the User none volatile memory immediately to File System.
  \return      none
*/
void slpManFlushUsrNVMem(void);
/**
  \fn          void slpManStartWaitATTimer(void);
  \brief       Once Receive AT Command, can use this funtion to add a delay before sleep. The delay is slpWaitTime
               Use AT+ECPCFG="slpWaitTime",4000  to set the slpWaitTime to 4000ms
  \return      none
*/
void slpManStartWaitATTimer(void);
/**
  \fn          void slpManWaitATTimerSet(uint16_t nMs, bool save2flash);
  \brief       Set the slpWaitTime used by slpManStartWaitATTimer function.
  \param[in]   nMs		   set slpWaitTime in micro second
  \param[in]   save2flash  choose whether to save slpWaitTime settings to flash or not. We suggest to save this setting, as in hib/slp2 the setting will lost
  \return      none
*/
void slpManWaitATTimerSet(uint16_t nMs, bool save2flash);

void slpManGetPMUSettings(void);
/**
  \fn          Deprecated, use slpManNormalIOVoltSet
               void slpManIOLevelSel(uint8_t sel);
  \brief       LDO_IO33 output voltage select. This register has two bits, bit#0 is from AON HW, bit#1 is from AON SW.
  			   We only config bit1 here. When LDO_IO33 = low bit0 is high. LDO_IO33 is high when floating

  			   bit1, bit0
				00: 1.8V (default) 01: 3.25V
				10: 2.8V 11: 2.8V
  \return      none
*/
void slpManIOLevelSel(uint8_t bit1);
/**
  \fn          void slpManNormalIOVoltSet(IOVoltageSel_t sel);
  \brief       select normal io voltage.
  \return      none
*/
void slpManNormalIOVoltSet(IOVoltageSel_t sel);
/**
  \fn          void slpManAONIOVoltSet(IOVoltageSel_t sel);
  \brief       select aon io voltage.
  \return      none
*/
void slpManAONIOVoltSet(IOVoltageSel_t sel);
/**
  \fn          void slpManSetPmuTimingCfg(pmuTimeCfg_t *cfg);
  \brief       Set Pmu Timing configure if presleep/postsleep callback takes too much time
  \param[in]   cfg          new configure
  \return      none
*/
void slpManSetPmuTimingCfg(pmuTimeCfg_t *cfg);
/**
  \fn          void slpManGetCurrentPmuTimingCfg(pmuTimeCfg_t *cfg);
  \brief       Get current Pmu Timing configure for changing and set back by calling slpManSetPmuTimingCfg
  \param[out]  cfg          current configure
  \return      none
*/
void slpManGetCurrentPmuTimingCfg(pmuTimeCfg_t *cfg);
/**
  \fn          void slpManRestoreDefaultPmuTimingCfg(void);
  \brief       Restore default pmu timing cfg
  \return      none
*/
void slpManRestoreDefaultPmuTimingCfg(void);
/**
  \fn          uint32_t slpManGetEstimateSlpTime(void)
  \brief       Get estimated sleep time, used in userdefined backup callback. 
  			   The actual sleep time may be a little bit shorter(less than 100ms)
  \return      slpTime in ms
*/
uint32_t slpManGetEstimateSlpTime(void);
/**
  \fn          void slpManAPPSetAONFlag1(bool flag);
  \brief       One bit flag for app to use, valid in all sleep mode
  			   when wakeup from hib/sleep2/sleep1, it is still the value before sleep
  \param[in]   flag          one bit flag for app use
  \return      none
*/
void slpManAPPSetAONFlag1(bool flag);
/**
  \fn          bool slpManAPPGetAONFlag1(void);
  \brief       Get one bit flag set by slpManAPPSetAONFlag1
  \return      flag
*/
bool slpManAPPGetAONFlag1(void);


////////// API Blow are for internal use only //////////////
bool slpManExtIntPreProcess(uint8_t bitmap);
/**
  \fn          uint8_t slpManExcutePredefinedBackupCb(slpManLpState state)
  \brief       Excute the predefined backup function.
  \param[in]   state          input the state of current sleep process
  \return      how many callback have been excute.
*/
uint8_t slpManExcutePredefinedBackupCb(slpManLpState state);
/**
  \fn          uint8_t slpManExcutePredefinedRestoreCb(slpManLpState state)
  \brief       Excute the predefined restore function.
  \param[in]   state          input the state of current sleep process
  \return      how many callback have been excute.
*/
uint8_t slpManExcutePredefinedRestoreCb(slpManLpState state);
/**
  \fn          uint8_t slpManExcuteUsrdefinedBackupCb(slpManLpState state)
  \brief       Excute the user defined backup function.
  \param[in]   state          input the state of current sleep process
  \return      how many callback have been excute.
*/
uint8_t slpManExcuteUsrdefinedBackupCb(slpManLpState state);
/**
  \fn          uint8_t slpManExcuteUsrdefinedRerstoreCb(slpManLpState state)
  \brief       Excute the user defined restore function.
  \param[in]   state          input the state of current sleep process
  \return      how many callback have been excute.
*/
uint8_t slpManExcuteUsrdefinedRestoreCb(slpManLpState state);
/**
  \fn          slpManRet_t slpManDrvVoteSleep(slpDrvVoteModule_t module, slpManSlpState_t status)
  \brief       drivers vote to sleep.
  \param[in]   module        input the vote module.
  \param[in]   status        input the sleep status, user should try to vote to the deepest state.
  \return      error code
*/
slpManRet_t slpManDrvVoteSleep(slpDrvVoteModule_t module, slpManSlpState_t status);

/**
  \fn          void slpManProductionTest(uint8_t mode)
  \brief       for current test for sleep mode in production line only
  \param[in]   mode    SLEEP1 = 0,
					   SLEEP2 = 1,
					   HIBERNATE1 = 2,
					   HIBERNATE2 = 3,
  \return      none
*/
void slpManProductionTest(uint8_t mode);

/**
  \fn          uint32_t slpManGetWakeupSlowCnt(void)
  \brief       Get wakeup slowcnt
  \return      wakeup slowcnt
*/
uint32_t slpManGetWakeupSlowCnt(void);

/**
  \fn          uint32_t slpManGetSleepTime(void)
  \brief       Get sleep time
  \return      sleep time
*/
uint32_t slpManGetSleepTime(void);
/**
  \fn          uint32_t slpManGetLatchPad(void)
  \brief       Get latched pad bitmap, bit0-bit5 -> pad0-pad5
  \return      sleep time
*/
uint8_t slpManGetLatchPad(void);
/**
  \fn          void slpManGetRawFlashRegionFlag(uint8_t *platPsRG, uint8_t *phyRG)
  \brief       Get the flash region prepare to write before sleep
  \return      sleep time
*/
void slpManGetRawFlashRegionFlag(uint8_t *platPsRG, uint8_t *phyRG);


/** \} */


#endif


/******************************************************************************
 *  KERNEL HEADER
 ******************************************************************************/
#include "sec_osal.h"

#include <linux/string.h>
#include <linux/bug.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/vmalloc.h>

/*****************************************************************************
 * MACRO
 *****************************************************************************/
#ifndef ASSERT
#define ASSERT(expr)        BUG_ON(!(expr))
#endif

/*****************************************************************************
 * GLOBAL VARIABLE
 *****************************************************************************/
DEFINE_SEMAPHORE(hacc_sem);
DEFINE_SEMAPHORE(mtd_sem);
DEFINE_SEMAPHORE(rid_sem);
DEFINE_SEMAPHORE(sec_mm_sem);
DEFINE_SEMAPHORE(osal_fp_sem);
DEFINE_SEMAPHORE(osal_verify_sem);
DEFINE_SEMAPHORE(osal_secro_sem);
DEFINE_SEMAPHORE(osal_secro_v5_sem);

/*****************************************************************************
 * LOCAL VARIABLE
 *****************************************************************************/

/*****************************************************************************
 * PORTING LAYER
 *****************************************************************************/
void osal_kfree(void *buf)
{
/* kfree(buf); */
	vfree(buf);
}
EXPORT_SYMBOL(osal_kfree);

void *osal_kmalloc(unsigned int size)
{
/* return kmalloc(size,GFP_KERNEL); */
	return vmalloc(size);
}
EXPORT_SYMBOL(osal_kmalloc);

unsigned long osal_copy_from_user(void *to, void *from, unsigned long size)
{
	return copy_from_user(to, from, size);
}
EXPORT_SYMBOL(osal_copy_from_user);

unsigned long osal_copy_to_user(void *to, void *from, unsigned long size)
{
	return copy_to_user(to, from, size);
}
EXPORT_SYMBOL(osal_copy_to_user);

int osal_hacc_lock(void)
{
	return down_interruptible(&hacc_sem);
}
EXPORT_SYMBOL(osal_hacc_lock);

void osal_hacc_unlock(void)
{
	up(&hacc_sem);
}
EXPORT_SYMBOL(osal_hacc_unlock);

int osal_verify_lock(void)
{
	return down_interruptible(&osal_verify_sem);
}
EXPORT_SYMBOL(osal_verify_lock);

void osal_verify_unlock(void)
{
	up(&osal_verify_sem);
}
EXPORT_SYMBOL(osal_verify_unlock);

int osal_secro_lock(void)
{
	return down_interruptible(&osal_secro_sem);
}
EXPORT_SYMBOL(osal_secro_lock);

void osal_secro_unlock(void)
{
	up(&osal_secro_sem);
}
EXPORT_SYMBOL(osal_secro_unlock);

int osal_secro_v5_lock(void)
{
	return down_interruptible(&osal_secro_v5_sem);
}
EXPORT_SYMBOL(osal_secro_v5_lock);

void osal_secro_v5_unlock(void)
{
	up(&osal_secro_v5_sem);
}
EXPORT_SYMBOL(osal_secro_v5_unlock);

int osal_mtd_lock(void)
{
	return down_interruptible(&mtd_sem);
}
EXPORT_SYMBOL(osal_mtd_lock);

void osal_mtd_unlock(void)
{
	up(&mtd_sem);
}
EXPORT_SYMBOL(osal_mtd_unlock);

int osal_rid_lock(void)
{
	return down_interruptible(&rid_sem);
}
EXPORT_SYMBOL(osal_rid_lock);

void osal_rid_unlock(void)
{
	up(&rid_sem);
}
EXPORT_SYMBOL(osal_rid_unlock);

void osal_msleep(unsigned int msec)
{
	msleep(msec);
}
EXPORT_SYMBOL(osal_msleep);

void osal_assert(unsigned int val)
{
	ASSERT(val);
}
EXPORT_SYMBOL(osal_assert);

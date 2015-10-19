#ifndef SEC_OSAL_H
#define SEC_OSAL_H

/**************************************************************************
 *  Operating System Abstract Layer - External Function
 **************************************************************************/
extern void osal_kfree(void *buf);
extern void *osal_kmalloc(unsigned int size);
extern unsigned long osal_copy_from_user(void *to, void *from, unsigned long size);
extern unsigned long osal_copy_to_user(void *to, void *from, unsigned long size);
extern int osal_hacc_lock(void);
extern void osal_hacc_unlock(void);
extern int osal_verify_lock(void);
extern void osal_verify_unlock(void);
extern int osal_secro_lock(void);
extern void osal_secro_unlock(void);
extern int osal_secro_v5_lock(void);
extern void osal_secro_v5_unlock(void);
extern int osal_mtd_lock(void);
extern void osal_mtd_unlock(void);
extern int osal_rid_lock(void);
extern void osal_rid_unlock(void);
extern void osal_msleep(unsigned int msec);
extern void osal_assert(unsigned int val);

/**************************************************************************
 *  Operating System Abstract Layer - Macro
 **************************************************************************/
#define SEC_ASSERT(a) osal_assert(a)

#define ASF_MALLOC(len) osal_kmalloc(len)
#define ASF_FREE(buf) osal_kfree(buf)
#define ASF_STRTOK(str, delim) strsep(&str, delim)

#endif				/* SEC_OSAL_H */

/*
 *  linux/arch/arm64/mm/fjsec_pageattr.c
 *
 *  Copyright (C) 2014-2015 Fujitsu, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/vmalloc.h>

#include <asm/cache.h>
#include <asm/cacheflush.h>
#include <asm/pgtable-hwdef.h>
#include <asm/pgtable.h>
#include <asm/sections.h>
#include <asm/tlbflush.h>
#include <asm/memory.h>

#include "mm.h"

//#define MEM_VIRT_ALIAS_CHK
//#define DEBUG_MEMATTR
//#define CONFIG_SECTIONMEM_ATTR_SUPPORT

/*
* Debug print
*/
#ifdef DEBUG_MEMATTR
#define memattr_debug(x, ...) printk(x, __VA_ARGS__)
#else //DEBUG_MEMATTR
#define memattr_debug(x, ...)
#endif //DEBUG_MEMATTR

enum {
	CH_ATTR_RDONLY,
	CH_ATTR_RW,
	CH_ATTR_NX,
	CH_ATTR_EXEC
};

typedef pte_t (* func_attr_pte)(pte_t*);
typedef pmd_t (* func_attr_pmd)(pmd_t*);

/*
 * Level 3 descriptor (PTE) handling for arm64. see pgtable-hwdef.h
 */
static inline pte_t pte_wrtprotect(pte_t* pte) {
	pte_val(*pte) |= PTE_RDONLY;
	pte_val(*pte) &= ~PTE_WRITE;
	return *pte;
}

static inline pte_t pte_mkwrt(pte_t* pte) {
	pte_val(*pte) &= ~PTE_RDONLY;
	pte_val(*pte) |= PTE_WRITE;
	return *pte;
}

static inline pte_t pte_mkpxnuxn(pte_t* pte) {
	pte_val(*pte) |= (PTE_PXN|PTE_UXN);
	return *pte;
}

static inline pte_t pte_mkexecex(pte_t* pte) {
	pte_val(*pte) &= ~(PTE_UXN);
	return *pte;
}

static const func_attr_pte pte_func[] = {
	pte_wrtprotect,
	pte_mkwrt,
	pte_mkpxnuxn,
	pte_mkexecex
};

static inline pmd_t pmd_mknexec(pmd_t* pmd) {
	pte_t tmp = pmd_pte(*pmd);
	return pte_pmd(pte_mkpxnuxn(&tmp));
}

static inline pmd_t pmd_mkexec(pmd_t* pmd) {
	pte_t tmp = pmd_pte(*pmd);
	return pte_pmd(pte_mkexecex(&tmp));
}

static inline pmd_t pmd_wrtprotect(pmd_t* pmd) {
	pte_t tmp = pmd_pte(*pmd);
	return pte_pmd(pte_wrtprotect(&tmp));
}

static inline pmd_t pmd_mkwrt(pmd_t* pmd) {
	pte_t tmp = pmd_pte(*pmd);
	return pte_pmd(pte_mkwrt(&tmp));
}

static const func_attr_pmd pmd_func[] = {
	pmd_wrtprotect,
	pmd_mkwrt,
	pmd_mknexec,
	pmd_mkexec
};

static int set_pte_attr(pmd_t *pmd, unsigned long start, unsigned long end, int mode)
{
	pte_t pte_mod;
	pte_t *pte = pte_offset_kernel(pmd, start);
	unsigned long addr = start;

	if( pte == NULL || pte_val(*pte) == 0 || pte_none(*pte) || !pte_present(*pte) ) return -EINVAL;

	arch_enter_lazy_mmu_mode();

	do {
		if( pte_val(*pte) == 0 || pte_none(*pte) || !pte_present(*pte) ) continue;
		pte_mod = pte_func[mode](pte);
	} while (pte++, addr += PAGE_SIZE, addr != end);

	arch_leave_lazy_mmu_mode();

	return 0;
}

static int walk_pmd(pud_t *pud, unsigned long start, unsigned long end, int mode)
{
	pmd_t pmd_mod;
	unsigned long next;
	pmd_t *pmd = pmd_offset(pud, start);
	unsigned long addr = start;

	if (pmd == NULL || *pmd == 0 || pmd_val(*pmd) == 0 || pmd_none(*pmd) || !pmd_present(*pmd) || pmd_bad(*pmd)) return -EINVAL;

	do {
		next = pmd_addr_end(addr, end);

		if ( pmd_val(*pmd) == 0 || pmd_none(*pmd) || !pmd_present(*pmd) || pmd_bad(*pmd)) continue;

		if (pmd_sect(*pmd)) {
			pmd_mod = pmd_func[mode](pmd);
			set_pmd(pmd, pmd_mod);
			continue;
		}

		if( set_pte_attr(pmd, addr, end, mode) ) return -EINVAL;
	} while (pmd++, addr = next, addr != end);

	return 0;
}

static int walk_pud(pgd_t *pgd, unsigned long start, unsigned long end, int mode)
{
	unsigned long next;
	pud_t *pud = pud_offset(pgd, start);
	unsigned long addr = start;

	if (pud == NULL || pud_val(*pud) == 0 || pud_none(*pud) ) return -EINVAL;

	do {
		next = pud_addr_end(addr, end);
		if( walk_pmd(pud, addr, end, mode) ) return -EINVAL;
	} while (pud++, addr = next, addr != end);

	return 0;
}

static int lookup_pte_set(unsigned long start, unsigned long end, int mode)
{
	unsigned long next;
	pgd_t *pgd = pgd_offset_k(start);
	unsigned long addr = start;

	if (pgd == NULL || pgd_val(*pgd) == 0 || pgd_none(*pgd) || addr > end) return -EINVAL;

	do {
		next = pgd_addr_end(addr, end);
		if( walk_pud(pgd, addr, next, mode) ) return -EINVAL;
	} while (pgd++, addr = next, addr != end);

	return 0;
}

static int change_page_attributes(unsigned long virt, unsigned long numpages, int mode)
{
	int ret = 0;
	unsigned long start;
	unsigned long end;

	memattr_debug("chpg_attr vaddr=%16lX, numpage=%lu, mode=%d\n", virt, numpages, mode);
	
	/* Alignment check */
	if (virt & ~PAGE_MASK) {
		virt &= PAGE_MASK;
	}

	/* Save start address in order to flash cache later */
	start = virt;
	end = virt + (numpages << PAGE_SHIFT);

	memattr_debug("chpg_attr startv=%16lX, endv=%16lX\n", start, end);

	/*
	 * flush all unused kmap mappings in order to remove stray mappings
	 */
	kmap_flush_unused();

	/**
	 * vm_unmap_aliases - unmap outstanding lazy aliases in the vmap layer
	 *
	 * The vmap/vmalloc layer lazily flushes kernel virtual mappings primarily
	 * to amortize TLB flushing overheads. What this means is that any page you
	 * have now, may, in a former life, have been mapped into kernel virtual
	 * address by the vmap layer and so there might be some CPUs with TLB entries
	 * still referencing that page (additional to the regular 1:1 kernel mapping).
	 *
	 * vm_unmap_aliases flushes all such lazy mappings. After it returns, we can
	 * be sure that none of the pages we have control over will have any aliases
	 * from the vmap layer.
	 */
	vm_unmap_aliases();

	/*
	 * If 4k page entry is found, then set attributes.
	 */
	ret = lookup_pte_set(start, end, mode);

	/* flush tlb entry and cpu cache */
	flush_tlb_kernel_range(start, end);
	isb();

	return ret;
}


/*
* Spin lock def
*/
static DEFINE_SPINLOCK(chgattr_lock);

/*
 * Lock the state
 */
static void change_page_attr_spinlock(unsigned long *flags)
{
	spin_lock_irqsave(&chgattr_lock, *flags);
}

/*
 * Unlock the state
 */
static void change_page_attr_spinunlock(unsigned long *flags)
{
	spin_unlock_irqrestore(&chgattr_lock, *flags);
}


/*
*   Change the virtual memory attributes into readonly.
*   
*   Input: 
	unsigned long virt	:The virtual memory start address to change its attributes
	int numpages		:The number of the virtual memory page
*
*	pte_wrprotect function is defined in the "pgtable.h" such like "PTE_BIT_FUNC(wrprotect, |= L_PTE_RDONLY);"
*/
int set_memory_ro2(unsigned long virt, unsigned long numpages)
{
	unsigned long flag;
	int ret;

	change_page_attr_spinlock(&flag);
	ret = change_page_attributes(virt, numpages, CH_ATTR_RDONLY);
	change_page_attr_spinunlock(&flag);

	return ret;
}
EXPORT_SYMBOL(set_memory_ro2);

/*
*   Change the virtual memory attributes into read-writable.
*   
*   Input: 
	unsigned long virt	:The virtual memory start address to change its attributes
	int numpages		:The number of the virtual memory page
*
*	pte_mkwrite function is defined in the "pgtable.h" such like "PTE_BIT_FUNC(mkwrite,   &= ~L_PTE_RDONLY);"
*/
int set_memory_rw2(unsigned long virt, unsigned long numpages)
{
	unsigned long flag;
	int ret;

	change_page_attr_spinlock(&flag);
	ret = change_page_attributes(virt, numpages, CH_ATTR_RW);
	change_page_attr_spinunlock(&flag);

	return ret;
}
EXPORT_SYMBOL(set_memory_rw2);

/*
*   Change the virtual memory attributes into executable.
*   
*   Input: 
	unsigned long virt	:The virtual memory start address to change its attributes
	int numpages		:The number of the virtual memory page
*
*/
int set_memory_x2(unsigned long virt, unsigned long numpages)
{
	unsigned long flag;
	int ret;

	change_page_attr_spinlock(&flag);
	ret = change_page_attributes(virt, numpages, CH_ATTR_EXEC);
	change_page_attr_spinunlock(&flag);

	return ret;
}
EXPORT_SYMBOL(set_memory_x2);

/*
*   Change the virtual memory attributes into non-executable.
*   
*   Input: 
	unsigned long virt	:The virtual memory start address to change its attributes
	int numpages		:The number of the virtual memory page
*
*/
int set_memory_nx2(unsigned long virt, unsigned long numpages)
{
	unsigned long flag;
	int ret;

	change_page_attr_spinlock(&flag);
	ret = change_page_attributes(virt, numpages, CH_ATTR_NX);
	change_page_attr_spinunlock(&flag);

	return ret;
}
EXPORT_SYMBOL(set_memory_nx2);

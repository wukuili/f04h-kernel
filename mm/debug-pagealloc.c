/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2015
/*----------------------------------------------------------------------------*/
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/page-debug-flags.h>
#include <linux/poison.h>
#include <linux/ratelimit.h>

#ifndef mark_addr_rdonly
#define mark_addr_rdonly(a)
#endif

#ifndef mark_addr_rdwrite
#define mark_addr_rdwrite(a)
#endif

static inline void set_page_poison(struct page *page)
{
	__set_bit(PAGE_DEBUG_FLAG_POISON, &page->debug_flags);
}

static inline void clear_page_poison(struct page *page)
{
	__clear_bit(PAGE_DEBUG_FLAG_POISON, &page->debug_flags);
}

static inline bool page_poison(struct page *page)
{
	return test_bit(PAGE_DEBUG_FLAG_POISON, &page->debug_flags);
}

static void poison_page(struct page *page)
{
	void *addr = kmap_atomic(page);

	set_page_poison(page);
	memset(addr, PAGE_POISON, PAGE_SIZE);
	mark_addr_rdonly(addr);
	kunmap_atomic(addr);
}

static void poison_pages(struct page *page, int n)
{
	int i;

	for (i = 0; i < n; i++)
		poison_page(page + i);
}

static bool single_bit_flip(unsigned char a, unsigned char b)
{
	unsigned char error = a ^ b;

	return error && !(error & (error - 1));
}
/* FUJITSU LIMITED:2015-01-08 H1510167 mod start */
/* static void check_poison_mem(unsigned char *mem, size_t bytes) */
static void check_poison_mem(unsigned char *mem, size_t bytes, struct page *page)
/* FUJITSU LIMITED:2015-01-08 H1510167 mod end */
{
	static DEFINE_RATELIMIT_STATE(ratelimit, 5 * HZ, 10);
	unsigned char *start;
	unsigned char *end;
	unsigned char *phy_mem;	/* FUJITSU LIMITED:2015-01-08 H1510167 add */

	start = memchr_inv(mem, PAGE_POISON, bytes);
	if (!start)
		return;

	for (end = mem + bytes - 1; end > start; end--) {
		if (*end != PAGE_POISON)
			break;
	}

	if (!__ratelimit(&ratelimit))
		return;
	else if (start == end && single_bit_flip(*start, PAGE_POISON))
		printk(KERN_ERR "pagealloc: single bit error\n");
	else
		printk(KERN_ERR "pagealloc: memory corruption\n");

	/* FUJITSU LIMITED:2015-01-08 H1510167 add start */
	phy_mem = (unsigned char *)__pfn_to_phys(page_to_pfn(page));
	printk(KERN_ERR "virtual address 0x%p = physical address 0x%p\n", mem, phy_mem);
	/* FUJITSU LIMITED:2015-01-08 H1510167 add end */

	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 16, 1, start,
			end - start + 1, 1);
	BUG_ON(PANIC_CORRUPTION);
	dump_stack();
}

static void unpoison_page(struct page *page)
{
	void *addr;

	if (!page_poison(page))
		return;

	addr = kmap_atomic(page);
/* FUJITSU LIMITED:2015-01-08 H1510167 mod start */
/*	check_poison_mem(addr, PAGE_SIZE); */
	check_poison_mem(addr, PAGE_SIZE, page);
/* FUJITSU LIMITED:2015-01-08 H1510167 mod end */
	mark_addr_rdwrite(addr);
	clear_page_poison(page);
	kunmap_atomic(addr);
}

static void unpoison_pages(struct page *page, int n)
{
	int i;

	for (i = 0; i < n; i++)
		unpoison_page(page + i);
}

void kernel_map_pages(struct page *page, int numpages, int enable)
{
	if (enable)
		unpoison_pages(page, numpages);
	else
		poison_pages(page, numpages);
}

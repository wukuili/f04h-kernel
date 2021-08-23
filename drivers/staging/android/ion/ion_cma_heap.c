/*
 * drivers/gpu/ion/ion_cma_heap.c
 *
 * Copyright (C) Linaro 2012
 * Author: <benjamin.gaignard@linaro.org> for ST-Ericsson.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/msm_ion.h>

#include <asm/cacheflush.h>

#include "ion.h"
#include "ion_priv.h"

/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD S */
// For FIDO def
#include <linux/msm_ion_ex.h>

#ifdef FIDO_SEC
#ifdef CONFIG_SECURITY_FJSEC
extern int fjsec_set_ioninfo(void *cpu_addr, unsigned long len, dma_addr_t handle, const char* heapname, pid_t pid);
extern int fjsec_remove_ioninfo(void *cpu_addr, dma_addr_t handle);
extern int fjsec_ion_access_control(void *cpu_addr, unsigned long len, dma_addr_t handle, pid_t pid, const char* heapname, int skip);
#endif
#endif
/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD E */

#define ION_CMA_ALLOCATE_FAILED -1

struct ion_cma_buffer_info {
	void *cpu_addr;
	dma_addr_t handle;
	struct sg_table *table;
	bool is_cached;
/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD S */
#ifdef FIDO_SEC
	bool is_fido;
	pid_t pid;
#endif
/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD E */
};

static int cma_heap_has_outer_cache;
/*
 * Create scatter-list for the already allocated DMA buffer.
 * This function could be replace by dma_common_get_sgtable
 * as soon as it will avalaible.
 */
static int ion_cma_get_sgtable(struct device *dev, struct sg_table *sgt,
			       void *cpu_addr, dma_addr_t handle, size_t size)
{
	struct page *page = pfn_to_page(PFN_DOWN(handle));
	int ret;

	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (unlikely(ret))
		return ret;

	sg_set_page(sgt->sgl, page, PAGE_ALIGN(size), 0);
	return 0;
}

/* ION CMA heap operations functions */
static int ion_cma_allocate(struct ion_heap *heap, struct ion_buffer *buffer,
			    unsigned long len, unsigned long align,
			    unsigned long flags)
{
	struct device *dev = heap->priv;
	struct ion_cma_buffer_info *info;

	dev_dbg(dev, "Request buffer allocation len %ld\n", len);

	info = kzalloc(sizeof(struct ion_cma_buffer_info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "Can't allocate buffer info\n");
		return ION_CMA_ALLOCATE_FAILED;
	}

	if (!ION_IS_CACHED(flags))
		info->cpu_addr = dma_alloc_writecombine(dev, len,
					&(info->handle), GFP_KERNEL);
	else
		info->cpu_addr = dma_alloc_nonconsistent(dev, len,
					&(info->handle), GFP_KERNEL);

	if (!info->cpu_addr) {
		dev_err(dev, "Fail to allocate buffer\n");
		goto err;
	}

	info->table = kmalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!info->table) {
		dev_err(dev, "Fail to allocate sg table\n");
		goto err;
	}

	info->is_cached = ION_IS_CACHED(flags);

/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD S */
#ifdef FIDO_SEC
	info->is_fido = ION_IS_FIDO(flags);
	info->pid = task_pid_nr(current->group_leader);
	if( info->is_fido ) {
#ifdef CONFIG_SECURITY_FJSEC
		if( fjsec_set_ioninfo(info->cpu_addr, len, info->handle, heap->name, info->pid) ) {
			pr_info("%s: The caller cannot have access to the fido ion.\n", __func__);
			kfree(info->table);
			goto err;
		}
#endif
	}
#endif
/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD E */

	ion_cma_get_sgtable(dev,
			info->table, info->cpu_addr, info->handle, len);

	/* keep this for memory release */
	buffer->priv_virt = info;
	dev_dbg(dev, "Allocate buffer %pK\n", buffer);
	return 0;

err:
	kfree(info);
	return ION_CMA_ALLOCATE_FAILED;
}

static void ion_cma_free(struct ion_buffer *buffer)
{
	struct device *dev = buffer->heap->priv;
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	dev_dbg(dev, "Release buffer %pK\n", buffer);
/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD S */
#ifdef FIDO_SEC
	if( info->is_fido ) {
#ifdef CONFIG_SECURITY_FJSEC
		if( fjsec_remove_ioninfo(info->cpu_addr, info->handle) ) {
			pr_info("%s: The caller cannot remove the fido ion info.\n", __func__);
		}
#endif
	}
#endif
/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD E */
	/* release memory */
	dma_free_coherent(dev, buffer->size, info->cpu_addr, info->handle);
	sg_free_table(info->table);
	/* release sg table */
	kfree(info->table);
	kfree(info);
}

/* return physical address in addr */
static int ion_cma_phys(struct ion_heap *heap, struct ion_buffer *buffer,
			ion_phys_addr_t *addr, size_t *len)
{
	struct device *dev = heap->priv;
	struct ion_cma_buffer_info *info = buffer->priv_virt;

	dev_dbg(dev, "Return buffer %pK physical address 0x%pa\n", buffer,
		&info->handle);

/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD S */
#ifdef FIDO_SEC
	if( info->is_fido ) {
#ifdef CONFIG_SECURITY_FJSEC
		if( fjsec_ion_access_control(info->cpu_addr, buffer->size, info->handle, info->pid, heap->name, 0) ) {
			pr_info("%s: The caller cannot have access to the fido ion.\n", __func__);
			return -EPERM;
		}
#endif
	}
#endif
/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD E */
	*addr = info->handle;
	*len = buffer->size;

	return 0;
}

static struct sg_table *ion_cma_heap_map_dma(struct ion_heap *heap,
					     struct ion_buffer *buffer)
{
	struct ion_cma_buffer_info *info = buffer->priv_virt;

/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD S */
#ifdef FIDO_SEC
	if( info->is_fido ) {
#ifdef CONFIG_SECURITY_FJSEC
		if( fjsec_ion_access_control(info->cpu_addr, buffer->size, info->handle, info->pid, heap->name, 0) ) {
			pr_info("%s: The caller cannot have access to the fido ion.\n", __func__);
			return ERR_PTR(-EPERM);
		}
#endif
	}
#endif
/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD E */

	return info->table;
}

static void ion_cma_heap_unmap_dma(struct ion_heap *heap,
				   struct ion_buffer *buffer)
{
	return;
}

static int ion_cma_mmap(struct ion_heap *mapper, struct ion_buffer *buffer,
			struct vm_area_struct *vma)
{
	struct device *dev = buffer->heap->priv;
	struct ion_cma_buffer_info *info = buffer->priv_virt;

/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD S */
#ifdef FIDO_SEC
	if( info->is_fido ) {
		pr_info("%s: user mmaping from heap for fido %s disallowed\n",
		__func__, mapper->name);
		return -EPERM;
	}
#endif
/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD E */

	if (info->is_cached)
		return dma_mmap_nonconsistent(dev, vma, info->cpu_addr,
				info->handle, buffer->size);
	else
		return dma_mmap_writecombine(dev, vma, info->cpu_addr,
				info->handle, buffer->size);
}

static void *ion_cma_map_kernel(struct ion_heap *heap,
				struct ion_buffer *buffer)
{
	struct ion_cma_buffer_info *info = buffer->priv_virt;

/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD S */
#ifdef FIDO_SEC

	if( info->is_fido ) {
#ifdef CONFIG_SECURITY_FJSEC
		if( fjsec_ion_access_control(info->cpu_addr, buffer->size, info->handle, info->pid, heap->name, 0) ) {
			pr_info("%s: The caller cannot have access to the fido ion.\n", __func__);
			return ERR_PTR(-EPERM);
		}
#endif
	}
#endif
/* FUJITSU:2015-03-07 GEN-141-SCRU-ALT ADD E */

	return info->cpu_addr;
}

static void ion_cma_unmap_kernel(struct ion_heap *heap,
				 struct ion_buffer *buffer)
{
	return;
}

static int ion_cma_print_debug(struct ion_heap *heap, struct seq_file *s,
			const struct list_head *mem_map)
{
	if (mem_map) {
		struct mem_map_data *data;

		seq_printf(s, "\nMemory Map\n");
		seq_printf(s, "%16.s %14.s %14.s %14.s\n",
			   "client", "start address", "end address",
			   "size");

		list_for_each_entry(data, mem_map, node) {
			const char *client_name = "(null)";


			if (data->client_name)
				client_name = data->client_name;

			seq_printf(s, "%16.s 0x%14pa 0x%14pa %14lu (0x%lx)\n",
				   client_name, &data->addr,
				   &data->addr_end,
				   data->size, data->size);
		}
	}
	return 0;
}

static struct ion_heap_ops ion_cma_ops = {
	.allocate = ion_cma_allocate,
	.free = ion_cma_free,
	.map_dma = ion_cma_heap_map_dma,
	.unmap_dma = ion_cma_heap_unmap_dma,
	.phys = ion_cma_phys,
	.map_user = ion_cma_mmap,
	.map_kernel = ion_cma_map_kernel,
	.unmap_kernel = ion_cma_unmap_kernel,
	.print_debug = ion_cma_print_debug,
};

struct ion_heap *ion_cma_heap_create(struct ion_platform_heap *data)
{
	struct ion_heap *heap;

	heap = kzalloc(sizeof(struct ion_heap), GFP_KERNEL);

	if (!heap)
		return ERR_PTR(-ENOMEM);

	heap->ops = &ion_cma_ops;
	/* set device as private heaps data, later it will be
	 * used to make the link with reserved CMA memory */
	heap->priv = data->priv;
	heap->type = ION_HEAP_TYPE_DMA;
	cma_heap_has_outer_cache = data->has_outer_cache;
	return heap;
}

void ion_cma_heap_destroy(struct ion_heap *heap)
{
	kfree(heap);
}

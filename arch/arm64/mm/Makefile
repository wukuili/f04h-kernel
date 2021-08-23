obj-y				:= dma-mapping.o extable.o fault.o init.o \
				   cache.o copypage.o flush.o \
				   ioremap.o iomap.o mmap.o pgd.o mmu.o \
				   context.o proc.o pageattr.o
obj-$(CONFIG_HUGETLB_PAGE)	+= hugetlbpage.o
obj-$(CONFIG_ARM64_PTDUMP)	+= dump.o

# FCNT LIMITED:2015-10-26 SEC-KERNEL_HARDENING-001 add start
obj-$(CONFIG_STRICT_MEMORY_RWX_EX) += fjsec_pageattr.o
# FCNT LIMITED:2015-10-26 SEC-KERNEL_HARDENING-001 add end

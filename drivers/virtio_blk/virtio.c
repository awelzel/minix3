/* So this is kind of a generic virtio library.
 *
 * Ok, this is notorious, but it would be nice to not have the huge
 * amount of headers here that each and every other file has.
 */
#define _SYSTEM 1


#include "errno.h"			/* for OK... */
#include <machine/pci.h>		/* PCI_ILR, PCI_BAR... */
#include <minix/syslib.h>		/* umap, vumap, out, in, alloc_..*/
#include <minix/sysutil.h>		/* at least panic() */

#include <string.h>			/* memset() */


/* TODO: These should actually end up in a libvirtio or so */
#include "virtio.h"
#include "virtio_ring.h"

#include <assert.h>

/* FIXME: Should get these from the kernel? */
#ifndef PAGE_SIZE
#define PAGE_SIZE 4096
#endif

int virtio_find_dev(const u16_t subdevid)
{
	int r, devind;
	u16_t vid, did, sdid;
	
	/* FIXME: Can we safely call this all the time? */
	pci_init();
	r = pci_first_dev(&devind, &vid, &did);

	while (r > 0) {
		sdid = pci_attr_r16(devind, PCI_SUBDID);
		if (vid == VIRTIO_VENDOR_ID && sdid == subdevid)
			return devind;
		r = pci_next_dev(&devind, &vid, &did);
	}

	return -1;
}

int virtio_config(const int devind, struct virtio_config *cfg)
{
	u32_t base, size;
	u32_t guest_features = 0, host_features = 0;
	int iof, r;

	/* FIXME:If we call pci_reserve() can we "unreserve" it? */
	/* pci_reserve(devind); */

	if ((r = pci_get_bar(devind, PCI_BAR, &base, &size, &iof)) != OK)
		panic("%s: Could not get BAR", cfg->name);

	if (!iof)
		panic("%s: No IO space?", cfg->name);

	if (base & 0xFFFF0000)
		panic("%s: Port weird (%08x)", cfg->name, base);

	/* store the I/O port */
	cfg->port = base;

	/* Reset the device */
	virtio_write8(cfg, VIRTIO_DEV_STATUS_OFF, 0);

	/* Read in host features */
	host_features = virtio_read32(cfg, VIRTIO_HOST_F_OFF);

	for (int i = 0; i < cfg->num_features; i++) {
		struct virtio_feature *f = &cfg->features[i];
		
		/* prepare the features the driver supports */
		guest_features |= (f->guest_support << f->bit);
	
		/* just load the host feature int the struct */
		f->host_support =  ((host_features >> f->bit) & 1);
	}

	/* let the device know about our features */
	virtio_write32(cfg, VIRTIO_GUEST_F_OFF, guest_features);

	/* Read the length of the queues */
	for (int i = 0; i < cfg->num_queues; i++) {
		struct virtio_queue *q = &cfg->queues[i];
		/* select the queue */
		virtio_write16(cfg, VIRTIO_QSEL_OFF, i);
		q->num = virtio_read16(cfg, VIRTIO_QSIZE_OFF);
		printf("%s: queue[%02d] num=%d\n", cfg->name, i,
						       q->num);

		if (q->num & (q->num - 1))
			panic("%s: Queue %d num=%d not ^2", cfg->name,
							    i,
							    q->num);
	}


	/* Ack the device */
	virtio_write8(cfg, VIRTIO_DEV_STATUS_OFF, VIRTIO_STATUS_ACK);

	/* Read IRQ line */
	cfg->irq = pci_attr_r8(devind, PCI_ILR);
	printf("%s: IRQ: %d\n", cfg->name, cfg->irq);
	
	/* We panic anywhere above if something goes wrong */
	return 0;
}
static void allocate_queue(struct virtio_queue *q)
{
	/* So we need memory, a lot */
	q->ring_size = vring_size(q->num, PAGE_SIZE);

	q->vaddr = alloc_contig(q->ring_size, AC_ALIGN4K, &q->paddr);
	q->data = alloc_contig(sizeof(q->data[0]) * q->num, AC_ALIGN4K, NULL);

	if (!q->vaddr || !q->data)
		panic("No memory for us");

	printf("queue at v=%p p=%08lx page=%d\n", q->vaddr, q->paddr,
							    q->page);
	
	return;
}

static void init_queue(struct virtio_queue *q)
{
	memset(q->vaddr, 0, q->ring_size);
	memset(q->data, 0, sizeof(q->data[0]) * q->num);

	/* physical page in guest, that is */
	q->page = q->paddr / PAGE_SIZE;
	
	/* Set pointers in q->vring according to size */
	vring_init(&q->vring, q->num, q->vaddr, PAGE_SIZE);

	/* Everything's free at this point */
	for (int i = 0; i < q->num; i++) {
		q->vring.desc[i].flags = VRING_DESC_F_NEXT;
		q->vring.desc[i].next = (i + 1) & (q->num - 1);
	}

	q->free_num = q->num;
	q->free_head = 0;
	q->free_tail = q->num - 1;
	q->last_used = 0;

	return;
}

int virtio_alloc_queues(struct virtio_config *cfg)
{
	/* If the driver wants to allocate queues, it probably knows
	 * how to drive the device. Let the device know.
	 */
	virtio_write8(cfg, VIRTIO_DEV_STATUS_OFF, VIRTIO_STATUS_DRV);


	for (int i = 0; i < cfg->num_queues; i++) {
		struct virtio_queue *q = &cfg->queues[i];
		allocate_queue(q);
		init_queue(q);

		/* select queue */
		virtio_write16(cfg, VIRTIO_QSEL_OFF, i);
		/* let the device know about the address */
		virtio_write32(cfg, VIRTIO_QADDR_OFF, q->page);
	}

	/* Driver is ready to go! */
	virtio_write8(cfg, VIRTIO_DEV_STATUS_OFF, VIRTIO_STATUS_DRV_OK);

	return 0;
}

static int wants_kick(struct virtio_queue *q)
{
	assert(q != NULL);

	/* We kick every time, because we don't have a fallback.
	 *
	 * Usually something like that should be done:
	 *
	 *  return !(q->vring.used->flags & VRING_USED_F_NO_NOTIFY);
	 *
	 *  and then also kick when descriptors are low
	 */
	return 1;
}

static void kick_queue(struct virtio_config *cfg, int qidx)
{
	assert(0 <= qidx && qidx < cfg->num_queues);

	if (wants_kick(&cfg->queues[qidx]))
		virtio_write16(cfg, VIRTIO_QNOTFIY_OFF, qidx);

	return;
}

/*
 * Linux is doing indirect blocks here if possible.
 * Linux is also not kicking the queue.
 * And linux actually cares about the features...
 *
 * TODO: virtio_buf_desc is similar to what vumap is,
 *       except for the lower bytes. Maybe we can get
 *       rid of that stuff...
 */
int virtio_to_queue(struct virtio_config *cfg, int qidx,
			struct virtio_buf_desc *bufs, size_t num,
			void *data)
{
	struct virtio_queue *q = &cfg->queues[qidx];
	struct vring *vring = &q->vring;
	int i, count = 0, free_first;
	struct vring_desc *vd;

	assert(0 <= qidx && qidx <= cfg->num_queues);

	if (!data)
		panic("%s: NULL data received queue %d", cfg->name, qidx);

	/* Oh oh, no free descriptors */
	if (q->free_num < num)
		panic("%s: Out of descriptors queue %d", cfg->name, qidx);

	free_first = q->free_head;

	/* TODO Should add an abstraction layer for the ring stuff */
	for (i = free_first; count < num; count++) {

		/* The next free descriptor */
		vd = &vring->desc[i];
	
		/* The descriptor should be in linked in the free list,
		 * and have a next entry, or it should be the last
		 * descriptor there.
		 */
		assert(vd->flags & VRING_DESC_F_NEXT);

		vd->addr = bufs[count].phys;
		vd->len = bufs[count].len;

		/* Reset flags */
		vd->flags = VRING_DESC_F_NEXT;

		/* If writeable */
		if (bufs[count].write)
			vd->flags |= VRING_DESC_F_WRITE;

		i = vd->next;
	}

	/* Unset the next bit in the last descriptor */
	vd->flags = vd->flags & ~VRING_DESC_F_NEXT;

	/* i is the idx to the descriptor we did not use */
	q->free_head = i;
	q->free_num -= num;

	/* Next index for host is free_head */
	vring->avail->ring[vring->avail->idx % q->num] = free_first;

	/* data provided by the caller to identify this slot */
	q->data[free_first] = data;
	
	/* Make sure the host sees the new descriptors */
	__insn_barrier();

	vring->avail->idx += 1;

	/* Make sure the host sees the avail->idx */
	__insn_barrier();

	/* kick it! */
	kick_queue(cfg, qidx);
	return 0;
}


int virtio_from_queue(struct virtio_config *cfg, int qidx, void **data)
{
	struct virtio_queue *q;
	struct vring *vring;
	struct vring_used_elem *uel;
	struct vring_desc *vd;
	int count = 0;
	u16_t idx;

	assert(0 <= qidx && qidx < cfg->num_queues);

	q = &cfg->queues[qidx];
	vring = &q->vring;

	/* Make sure we see the changes */
	__insn_barrier();

	/* get the used element */
	uel = &q->vring.used->ring[q->last_used];

	/* update last_used index */
	q->last_used = (q->last_used + 1) % q->num;

	/* index of the used element */
	idx = uel->id % q->num;

	assert(q->data[idx] != NULL);

	/* Get the descriptor */
	vd = &vring->desc[idx];

	/* Unconditionally set the tail->next to the first used one */
	assert(vring->desc[q->free_tail].flags & VRING_DESC_F_NEXT);
	vring->desc[q->free_tail].next = idx;

	/* Find the last index, eventually there has to be one
	 * without a the next flag.
	 *
	 * FIXME: Protect from endless loop
	 */
	while (vd->flags & VRING_DESC_F_NEXT) {
		idx = vd->next;
		vd = &vring->desc[idx];
		count++;
	}
	
	/* Didn't count the last one */
	count++;

	/* So, idx points to the tail now, update the queue */
	q->free_tail = idx;
	assert(!(vd->flags & VRING_DESC_F_NEXT));

	/* We can always connect the tail with the head */
	vring->desc[q->free_tail].next = q->free_head;
	vring->desc[q->free_tail].flags = VRING_DESC_F_NEXT;

	q->free_num += count;

	assert(q->free_num <= q->num);
	
	*data = q->data[uel->id];
	q->data[uel->id] = NULL;

	return 0;
}

int virtio_had_irq(struct virtio_config *cfg)
{
	return virtio_read8(cfg, VIRTIO_ISR_STATUS_OFF) & 1;
}

void virtio_reset(struct virtio_config *cfg)
{
	virtio_write8(cfg, VIRTIO_DEV_STATUS_OFF, 0);
}


void virtio_irq_register(struct virtio_config *cfg)
{
	int r;
	if ((r = sys_irqsetpolicy(cfg->irq, 0, &cfg->irq_hook) != OK))
		panic("%s: Unable to register IRQ %d", cfg->name, r);
}

void virtio_irq_unregister(struct virtio_config *cfg)
{
	int r;
	if ((r = sys_irqrmpolicy(&cfg->irq_hook) != OK))
		panic("%s: Unable to unregister IRQ %d", cfg->name, r);
}

void virtio_irq_enable(struct virtio_config *cfg)
{
	int r;
	if ((r = sys_irqenable(&cfg->irq_hook) != OK))
		panic("%s Unable to enable IRQ %d", cfg->name, r);
}

void virtio_irq_disable(struct virtio_config *cfg)
{
	int r;
	if ((r = sys_irqdisable(&cfg->irq_hook) != OK))
		panic("%s: Unable to enable IRQ %d", cfg->name, r);
}

static int _supports(struct virtio_config *cfg, int bit, int host)
{
	for (int i = 0; i < cfg->num_features; i++) {
		struct virtio_feature *f = &cfg->features[i];

		if (f->bit == bit)
			return host ? f->host_support : f->guest_support;
	}

	panic("%s: Feature not found bit=%d", cfg->name, bit);
}

int virtio_host_supports(struct virtio_config *cfg, int bit)
{
	return _supports(cfg, bit, 1);
}

int virtio_guest_supports(struct virtio_config *cfg, int bit)
{
	return _supports(cfg, bit, 0);
}


/* Just some wrappers around sys_read */
#define VIRTIO_READ_XX(xx, suff)					\
u##xx##_t virtio_read##xx(struct virtio_config *cfg, off_t off)		\
{									\
	int r;								\
	u32_t ret;							\
	if ((r = sys_in##suff(cfg->port + off, &ret)) != OK)		\
		panic("%s: Read failed %d %d r=%d", cfg->name,		\
						    cfg->port,		\
						    off,		\
						    r);			\
									\
	return ret;							\
}
VIRTIO_READ_XX(32, l)
VIRTIO_READ_XX(16, w)
VIRTIO_READ_XX(8, b)

/* Just some wrappers around sys_write */
#define VIRTIO_WRITE_XX(xx, suff)					\
void virtio_write##xx(struct virtio_config *cfg, off_t off, u##xx##_t val)\
{									\
	int r;								\
	if ((r = sys_out##suff(cfg->port + off, val)) != OK)		\
		panic("%s: Write failed %d %d r=%d", cfg->name,		\
						     cfg->port,		\
						     off,		\
						     r);		\
}

VIRTIO_WRITE_XX(32, l)
VIRTIO_WRITE_XX(16, w)
VIRTIO_WRITE_XX(8, b)


	
/* Just some wrappers around sys_read */
#define VIRTIO_SREAD_XX(xx, suff)					\
u##xx##_t virtio_sread##xx(struct virtio_config *cfg, off_t off)	\
{									\
	int r;								\
	u32_t ret;							\
	off += VIRTIO_DEV_SPECIFIC_OFF; 				\
									\
	if (cfg->msi)							\
		off += VIRTIO_MSI_ADD_OFF;				\
									\
	if ((r = sys_in##suff(cfg->port + off, &ret)) != OK)		\
		panic("%s: Read failed %d %d r=%d", cfg->name,		\
						    cfg->port,		\
						    off,		\
						    r);			\
									\
	return ret;							\
}

VIRTIO_SREAD_XX(32, l)
VIRTIO_SREAD_XX(16, w)
VIRTIO_SREAD_XX(8, b)

/* Just some wrappers around sys_write */
#define VIRTIO_SWRITE_XX(xx, suff)					\
void virtio_swrite##xx(struct virtio_config *cfg, off_t off, u##xx##_t val)\
{									\
	int r;								\
	off += VIRTIO_DEV_SPECIFIC_OFF; 				\
									\
	if (cfg->msi)							\
		off += VIRTIO_MSI_ADD_OFF;				\
									\
	if ((r = sys_out##suff(cfg->port + off, val)) != OK)		\
		panic("%s: Write failed %d %d r=%d", cfg->name,		\
						     cfg->port,		\
						     off,		\
						     r);		\
}

VIRTIO_SWRITE_XX(32, l)
VIRTIO_SWRITE_XX(16, w)
VIRTIO_SWRITE_XX(8, b)

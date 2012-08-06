#ifndef _VIRTIO_H
#define _VIRTIO_H
#include <sys/types.h>

/* This should actually be somwehere "official" */
#include "virtio_ring.h"

#define VIRTIO_VENDOR_ID			0x1AF4
#define VIRTIO_BLK_SUBDEV_ID			0x0002


#define VIRTIO_HOST_F_OFF			0x0000
#define VIRTIO_GUEST_F_OFF			0x0004
#define VIRTIO_QADDR_OFF			0x0008

#define VIRTIO_QSIZE_OFF			0x000C
#define VIRTIO_QSEL_OFF				0x000E
#define VIRTIO_QNOTFIY_OFF			0x0010

#define VIRTIO_DEV_STATUS_OFF			0x0012
#define VIRTIO_ISR_STATUS_OFF			0x0013
#define VIRTIO_DEV_SPECIFIC_OFF			0x0014
/* if msi is enabled, device specific headers shift by 4 */
#define VIRTIO_MSI_ADD_OFF			0x0004
#define VIRTIO_STATUS_ACK			0x01
#define VIRTIO_STATUS_DRV			0x02
#define VIRTIO_STATUS_DRV_OK			0x04
#define VIRTIO_STATUS_FAIL			0x80	/* 128 */


/* TODO: Better pointer types? */
struct virtio_buf_desc {
	u64_t phys;
	u32_t len;
	u8_t write;
};

struct virtio_queue {
	
	void *vaddr;				/* virtual addr of ring */
	unsigned long paddr;			/* physical addr of ring */
	u32_t page;				/* guest page */

	u16_t num;				/* number of descriptors */
	u32_t ring_size;			/* size of ring in bytes */
	struct vring vring;
	
	u16_t free_num;				/* free descriptors */
	u16_t free_head;			/* next free descriptor */
	u16_t free_tail;			/* last free descriptor */
	u16_t last_used;			/* we checked in used */

	void **data;				/* points to pointers */
};

/* Feature description */
struct virtio_feature {
	const char *name;
	u8_t bit;
	u8_t host_support;
	u8_t guest_support;
};

/* Generic virtio config */
struct virtio_config {

	char *name;				/* for debugging */
	
	u16_t  port;				/* io port from pci */

	struct virtio_feature *features;	/* host / guest features */
	u8_t num_features;			/* max 32 */

	struct virtio_queue *queues;		/* our queues */
	u16_t num_queues;
	
	int irq;				/* interrupt line */
	int irq_hook;				/* hook id */
	int msi;

	void *priv_conf;			/* private conf area */
	size_t conf_len;
	
};

int virtio_find_dev(const u16_t subdevid);

int virtio_config(const int pci_devind, struct virtio_config *cfg);

/* Allocate memory for all queues and the ring */
int virtio_alloc_queues(struct virtio_config *cfg);

int virtio_guest_supports(struct virtio_config *cfg, int bit);
int virtio_host_supports(struct virtio_config *cfg, int bit);

int virtio_to_queue(struct virtio_config *cfg, int qidx,
			struct virtio_buf_desc *bufs, size_t num,
			void *data);
int virtio_from_queue(struct virtio_config *cfg, int qidx, void **data);

void virtio_irq_register(struct virtio_config *cfg);
void virtio_irq_unregister(struct virtio_config *cfg);
void virtio_irq_enable(struct virtio_config *cfg);
void virtio_irq_disable(struct virtio_config *cfg);

/* Checks the ISR field of the device and returns true if
 * the interrupt was for this device.
 */
int virtio_had_irq(struct virtio_config *cfg);

void virtio_reset(struct virtio_config *cfg);

u32_t virtio_read32(struct virtio_config *cfg, off_t offset);
u16_t virtio_read16(struct virtio_config *cfg, off_t offset);
u8_t virtio_read8(struct virtio_config *cfg, off_t offset);
void virtio_write32(struct virtio_config *cfg, off_t offset, u32_t val);
void virtio_write16(struct virtio_config *cfg, off_t offset, u16_t val);
void virtio_write8(struct virtio_config *cfg, off_t offset, u8_t val);


/* Device specific reads take MSI offset into account and all reads
 * are at offset 20.
 *
 * Something like:
 * read(off) -->
 * 	readX(20 + (msi ? 4 : 0) + off)
 */
u32_t virtio_sread32(struct virtio_config *cfg, off_t offset);
u16_t virtio_sread16(struct virtio_config *cfg, off_t offset);
u8_t virtio_sread8(struct virtio_config *cfg, off_t offset);
void virtio_swrite32(struct virtio_config *cfg, off_t offset, u32_t val);
void virtio_swrite16(struct virtio_config *cfg, off_t offset, u16_t val);
void virtio_swrite8(struct virtio_config *cfg, off_t offset, u8_t val);

#endif /* _VIRTIO_H */
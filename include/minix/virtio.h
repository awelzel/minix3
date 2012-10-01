#ifndef SYS_VIRTIO_H
#define SYS_VIRTIO_H
#include <sys/types.h>

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


/* Feature description */
struct virtio_feature {
	const char *name;
	u8_t bit;
	u8_t host_support;
	u8_t guest_support;
};

/* Forward declaration virtio_queue */
struct virtio_queue;

/* Forward declaration virtio_config */
/* FIXME: This should be renamed to virtio_device or so */
struct virtio_config;


/* Find a virtio device with subdevice id subdevid. Returns a pointer
 * to an opaque virtio_config instance.
 */
struct virtio_config *virtio_setup_device(
		u16_t subdevid,
		const char *name,		/* only for debugging */
		int queue_count,
		struct virtio_feature *features, int feature_count,
		int threads,
		int skip			/* pci devices to skip */
);

/* Reset a virtio device */
void virtio_reset_device(struct virtio_config *cfg);


/* Allocate memory for all queues and the ring */
int virtio_alloc_queues(struct virtio_config *cfg);

int virtio_guest_supports(struct virtio_config *cfg, int bit);
int virtio_host_supports(struct virtio_config *cfg, int bit);

/*
 * Use num vumap_phys elements and chain these as vring_desc elements
 * into the vring.
 *
 * Kick the queue if needed.
 *
 * data is opaque and returned by virtio_from_queue() when the host
 * processed the descriptor chain.
 *
 * Note: The last bit of vp_addr is used to flag whether an iovec is
 * 	 writable. This implies that only word aligned buffers can be
 * 	 used.
 */
int virtio_to_queue(struct virtio_config *cfg, int qidx,
			struct vumap_phys *bufs, size_t num, void *data);

/*
 * If the host used a chain of descriptors, return 0 and set data
 * as was given to virtio_to_queue(). If the host has not processed
 * any element returns -1.
 */
int virtio_from_queue(struct virtio_config *cfg, int qidx, void **data);

/* IRQ related functions */
void virtio_irq_enable(struct virtio_config *cfg);
void virtio_irq_disable(struct virtio_config *cfg);

/* Checks the ISR field of the device and returns true if
 * the interrupt was for this device.
 */
int virtio_had_irq(struct virtio_config *cfg);

/* Config changes are propagated in the ISR field as well
 *
 * TODO: Implement
 */
int virtio_had_config_change(struct virtio_config *cfg);


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

#endif /* SYS_VIRTIO_H */

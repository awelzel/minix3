/* 
 * Poor man's approach to virtio-blk
 */

#include <assert.h>

#include <minix/drivers.h>
#include <minix/blockdriver_mt.h>
#include <minix/drvlib.h>
#include <minix/virtio.h>
#include <minix/sysutil.h>

#include <sys/ioc_disk.h>

#include "virtio_blk.h"

#define dprintf(s) do {						\
	printf("%s: ", name);					\
	printf s;						\
	printf("\n");						\
} while (0);

struct virtio_feature blkf[] = {
	/* name ,	bit,			host,	guest	*/
	{ "barrier",	VIRTIO_BLK_F_BARRIER,	0,	0 	},
	{ "sizemax",	VIRTIO_BLK_F_SIZE_MAX,	0,	0	},
	{ "segmax",	VIRTIO_BLK_F_SEG_MAX,	0,	0	},
	{ "geometry",	VIRTIO_BLK_F_GEOMETRY,	0,	0	},
	{ "read-only",	VIRTIO_BLK_F_RO,	0,	0	},
	{ "blocksize",	VIRTIO_BLK_F_BLK_SIZE,	0,	0	},
	{ "scsi",	VIRTIO_BLK_F_SCSI,	0,	0	},
	{ "flush",	VIRTIO_BLK_F_FLUSH,	0,	0	},
	{ "topology",	VIRTIO_BLK_F_TOPOLOGY,	0,	0	},
	{ "idbytes",	VIRTIO_BLK_ID_BYTES,	0,	0	}
};

static const char *const name = "virtio-blk";
static struct virtio_blk_config blk_config;
static struct virtio_config *config;

static int spurious_interrupt = 0;

static int terminating = 0;

static int open_count = 0;

#define VIRTIO_BLK_SUB_PER_DRIVE	(NR_PARTITIONS * NR_PARTITIONS)
#define VIRTIO_BLK_BLOCK_SIZE		512

#define VIRTIO_BLK_NUM_THREADS		4

struct device part[DEV_PER_DRIVE];			/* partition */
struct device subpart[VIRTIO_BLK_SUB_PER_DRIVE];	/* subpartitions */


/* Use for actual requests */
static struct virtio_blk_outhdr hdrs[VIRTIO_BLK_NUM_THREADS];

/* So usually a status is only one byte, but we have
 * some alignment issues if we dont take 2 bytes.
 */
static u16_t status[VIRTIO_BLK_NUM_THREADS];

/* Physical addresses */
static struct vumap_phys umap_hdrs[VIRTIO_BLK_NUM_THREADS];
static struct vumap_phys umap_status[VIRTIO_BLK_NUM_THREADS];

/* Prototypes */
static int virtio_open(dev_t minor, int access);
static int virtio_close(dev_t minor);
static ssize_t virtio_transfer(dev_t minor, int do_write, u64_t position,
	endpoint_t endpt, iovec_t *iovec, unsigned int count, int flags);
static int virtio_ioctl(dev_t minor, unsigned int request, endpoint_t endpt,
	cp_grant_id_t grant);
static struct device *virtio_part(dev_t minor);
static void virtio_geometry(dev_t minor, struct partition *entry);
static void virtio_intr(unsigned int UNUSED(irqs));
static int virtio_device(dev_t minor, device_id_t *id);
static void virtio_terminate(void);


static struct blockdriver virtio_blk_dtab  = {
	BLOCKDRIVER_TYPE_DISK,
	virtio_open,
	virtio_close,
	virtio_transfer,
	virtio_ioctl,
	NULL,		/* bdr_cleanup */
	virtio_part,
	virtio_geometry,
	virtio_intr,
	NULL,		/* bdr_alarm */
	NULL,		/* bdr_other */
	virtio_device
};

static int
virtio_open(dev_t minor, int access)
{
	/* Read only devices should only be mounted... read-only */
	if ((access & W_BIT) && virtio_host_supports(config, VIRTIO_BLK_F_RO))
		return EACCES;

	/* Partition magic when opened the first time */
	if (open_count == 0) {
		memset(part, 0, sizeof(part));
		memset(subpart, 0, sizeof(subpart));
		part[0].dv_size = blk_config.capacity * VIRTIO_BLK_BLOCK_SIZE;
		partition(&virtio_blk_dtab, 0, P_PRIMARY, 0 /* ATAPI */);
		blockdriver_mt_set_workers(0, VIRTIO_BLK_NUM_THREADS);
	}

	open_count++;
	return OK;
}

static int
virtio_close(dev_t minor)
{
	if (open_count == 0)
		return EINVAL;

	open_count--;

	if (open_count == 0 && terminating)
		virtio_terminate();

	return OK;
}

static int
check_addr_set_write_perm(struct vumap_vir *vir, struct vumap_phys *phys,
	int cnt, int write)
{

	for (int i = 0; i < cnt ; i++) {

		/* So you gave us a byte aligned buffer? Good job! */
		if (phys[i].vp_addr & 1) {
			dprintf(("odd buffer from %08lx", phys[i+1].vp_addr));
			return EINVAL;
		}

		/* Check if the buffer is good */
		if (phys[i].vp_size != vir[i].vv_size) {
			dprintf(("Non-contig buf %08lx", phys[i+1].vp_addr));
			return EINVAL;
		}

		/* If write, the buffers only need to be read */
		phys[i].vp_addr |= !write;
	}

	return OK;
}

static int
prepare_vir_vec(endpoint_t endpt, struct vumap_vir *vir, iovec_s_t *iv,
	int cnt, vir_bytes *size)
{
	/* This is pretty much the same as sum_iovec from AHCI,
	 * except that we don't support any iovecs where the size
	 * is not a multiple of 512
	 */
	vir_bytes tmp, total = 0;
	for (int i = 0; i < cnt; i++) {
		tmp = iv[i].iov_size;

		if (tmp == 0 || (tmp % VIRTIO_BLK_BLOCK_SIZE) || tmp > LONG_MAX) {
			dprintf(("bad iv[%d].iov_size (%lu) from %d", i, tmp,
									 endpt));
			return EINVAL;
		}

		total += tmp;

		if (total > LONG_MAX) {
			dprintf(("total overflow from %d", endpt));
			return EINVAL;
		}

		if (endpt == SELF)
			vir[i].vv_addr = (vir_bytes)iv[i].iov_grant;
		else
			vir[i].vv_grant = iv[i].iov_grant;

		vir[i].vv_size = iv[i].iov_size;

	}

	*size = total;
	return OK;
}

static ssize_t
virtio_transfer(dev_t minor, int write, u64_t position, endpoint_t endpt,
	iovec_t *iovec, unsigned int cnt, int flags)
{
	/* Need to translate vir to phys */
	struct vumap_vir vir[NR_IOREQS];

	/* Physical including header and trailer */
	struct vumap_phys phys[NR_IOREQS + 2];

	/* Which thread is doing this? */
	thread_id_t tid = blockdriver_mt_get_tid();

	/* header and trailer */
	vir_bytes size = 0;
	vir_bytes size_tmp = 0;
	struct device *dv;
	u64_t sector;
	u64_t end_part;
	int r, pcnt = sizeof(phys) / sizeof(phys[0]);

	iovec_s_t *iv = (iovec_s_t *)iovec;
	int access = write ? VUA_READ : VUA_WRITE;

	/* Don't touch this one anymore */
	iovec = NULL;

	if (cnt > NR_IOREQS)
		return EINVAL;

	/* position greater than capacity? */
	if (position >= blk_config.capacity * VIRTIO_BLK_BLOCK_SIZE)
		return 0;

	dv = virtio_part(minor);

	if (!dv)
		return ENXIO;


	position += dv->dv_base;
	end_part = dv->dv_base + dv->dv_size;

	/* Hmmm, AHCI tries to fix this up, but lets just say everything
	 * needs to be sector (512 byte ) aligned...
	 */
	if (position % VIRTIO_BLK_BLOCK_SIZE) {
		dprintf(("Non sector-aligned access %016llx", position));
		return EINVAL;
	}

	sector = position / VIRTIO_BLK_BLOCK_SIZE;

	r = prepare_vir_vec(endpt, vir, iv, cnt, &size);

	if (r != OK)
		return r;

	if (position == end_part)
		return 0;

	if (position > end_part)
		return 0;

	/* Truncate if the partition is smaller than that */
	if (position + size > end_part - 1) {
		size = end_part - position;

		/* Fix up later */
		size_tmp = 0;
		cnt = 0;
	} else {
		/* Use all buffers */
		size_tmp = size;
	}

	/* Fix up the number of vectors if size was truncated */
	while (size_tmp < size)
		size_tmp += vir[cnt++].vv_size;

	/* If the last vector was too big, just truncate it */
	if (size_tmp > size) {
		vir[cnt - 1].vv_size = vir[cnt -1].vv_size - (size_tmp - size);
		size_tmp -= (size_tmp - size);
	}

	if (size % VIRTIO_BLK_BLOCK_SIZE) {
		dprintf(("non-sector sized read (%lu) from %d", size, endpt));
		return EINVAL;
	}

	/* Map vir to phys */
	if ((r = sys_vumap(endpt, vir, cnt, 0, access,
			   &phys[1], &pcnt)) != OK) {

		dprintf(("Unable to map memory from %d (%d)", endpt, r));
		return r;
	}

	/* First the header */
	phys[0].vp_addr = umap_hdrs[tid].vp_addr;
	phys[0].vp_size = sizeof(hdrs[0]);

	r = check_addr_set_write_perm(vir, &phys[1], pcnt, write);

	if (r != OK)
		return r;

	/* Put the status at the end */
	phys[pcnt + 1].vp_addr = umap_status[tid].vp_addr;
	phys[pcnt + 1].vp_size = sizeof(u8_t);
	assert(!(phys[pcnt +  1].vp_addr & 1));

	/* Status always needs write access */
	phys[1 + pcnt].vp_addr |= 1;

	/* Prepare the header */
	if (write)
		hdrs[tid].type = VIRTIO_BLK_T_OUT;
	else
		hdrs[tid].type = VIRTIO_BLK_T_IN;

	hdrs[tid].ioprio = 0;
	hdrs[tid].sector = sector;


	/* Set status to "failure" */
	status[tid] = 0xFFFF;

	/* Send addresses to queue */
	virtio_to_queue(config, 0, phys, 2 + pcnt, &tid);

	/* Wait for completion */
	blockdriver_mt_sleep();

	/* Check status, only the lower 8 bits */
	if (status[tid] & 0xFF) {
		dprintf(("ERR status=%02x", status[tid] & 0xFF));
		dprintf(("ERR sector=%u size=%lx w=%d cnt=%d tid=%d",
			(u32_t) sector, size, write, pcnt, tid));

		return EIO;
	}

	return size;
}

static int
virtio_ioctl(dev_t minor, unsigned int request, endpoint_t endpt,
	cp_grant_id_t grant)
{
	switch (request) {

	case DIOCOPENCT:
		return sys_safecopyto(endpt, grant, 0,
			(vir_bytes) &open_count, sizeof(open_count));

	case DIOCFLUSH:
		if (!virtio_host_supports(config, VIRTIO_BLK_F_FLUSH)) {

			/* TODO: Implement */
			return EIO;
		}

		dprintf(("Host has no flush support"));

	}

	return EINVAL;
}


static struct device *
virtio_part(dev_t minor)
{
	/* There's only a single drive attached to this
	 * controller, so take some shortcuts.
	 */

	/* Take care of d0 d0p0 ... */
	if (minor < 5)
		return &part[minor];

	/* subparts start at 128 */
	if (minor >= 128) {

		/* Mask away upper bits */
		minor = minor & 0x7F;

		/* Only for the first disk */
		if (minor > 15)
			return NULL;

		return &subpart[minor];
	}

	return NULL;
}

static void
virtio_geometry(dev_t minor, struct partition *entry)
{
	/* Only for the drive */
	if (minor != 0)
		return;

	/* Only if the host supports it */
	if(!virtio_host_supports(config, VIRTIO_BLK_F_GEOMETRY))
		return;

	entry->cylinders = blk_config.geometry.cylinders;
	entry->heads = blk_config.geometry.heads;
	entry->sectors = blk_config.geometry.sectors;
}

static void
virtio_intr(unsigned int UNUSED(irqs))
{
	thread_id_t *ret_tid;

	if (virtio_had_irq(config)) {

		/* Multiple requests might have finished for a single
		 * interrupt.
		 */
		while (!virtio_from_queue(config, 0, (void**)&ret_tid)) {

			/* Wake up the responsible thread */
			blockdriver_mt_wakeup(*ret_tid);
		}
	} else if (!spurious_interrupt) {
		spurious_interrupt = 1;
		dprintf(("Got spurious interrupt"));
	}

	virtio_irq_enable(config);
}

static int
virtio_device(dev_t minor, device_id_t *id)
{
	struct device *dev = virtio_part(minor);

	/* Check if this device exists */
	if (dev == NULL)
		return ENXIO;

	*id = 0;
	return OK;
}

static void
virtio_terminate(void)
{
	/* Don't terminate if still opened */
	if (open_count > 0)
		return;

	virtio_reset_device(config);
	blockdriver_mt_terminate();
}

static void
virtio_blk_phys_mapping(void)
{
	/* Hack to get the physical addresses of hdr and status */
	struct vumap_vir virs[VIRTIO_BLK_NUM_THREADS];

	int r, cnt, pcnt;
	pcnt = cnt = VIRTIO_BLK_NUM_THREADS;
	int access = VUA_READ  | VUA_WRITE;

	/* prepare array for hdr addresses */
	for (int i = 0; i < VIRTIO_BLK_NUM_THREADS; i++) {
		virs[i].vv_addr = (vir_bytes)&hdrs[i];
		virs[i].vv_size = sizeof(hdrs[0]);
	}

	/* make the call */
	r = sys_vumap(SELF, virs, cnt, 0, access, umap_hdrs, &pcnt);

	if (r != OK)
		panic("%s: Unable to map hdrs %d", name, r);

	/* prepare array for status addresses */
	for (int i = 0; i < VIRTIO_BLK_NUM_THREADS; i++) {
		assert(!((vir_bytes)(&status[i]) & 1));
		virs[i].vv_addr = (vir_bytes)&status[i];
		virs[i].vv_size = sizeof(status[0]);
	}

	pcnt = cnt = VIRTIO_BLK_NUM_THREADS;

	r = sys_vumap(SELF, virs, cnt, 0, access, umap_status, &pcnt);

	if (r != OK)
		panic("%s: Unable to map status %d", name, r);
}

static int
virtio_blk_feature_setup(void)
{
	/* Feature setup for virtio_blk
	 *
	 * FIXME: This only reads and prints stuff right now, not taking
	 * any into account.
	 *
	 * We use virtio_sread() here, to jump over the generic
	 * headers using magic numbers...
	 */
	if (virtio_host_supports(config, VIRTIO_BLK_F_SEG_MAX)) {
		blk_config.seg_max = virtio_sread32(config, 12);
		dprintf(("Seg Max: %d", blk_config.seg_max));
	}

	if (virtio_host_supports(config, VIRTIO_BLK_F_GEOMETRY)) {
		blk_config.geometry.cylinders = virtio_sread16(config, 16);
		blk_config.geometry.heads= virtio_sread8(config, 18);
		blk_config.geometry.sectors = virtio_sread8(config, 19);
		
		dprintf(("Geometry: cyl=%d heads=%d sectors=%d",
					blk_config.geometry.cylinders,
					blk_config.geometry.heads,
					blk_config.geometry.sectors));
	}

	if (virtio_host_supports(config, VIRTIO_BLK_F_SIZE_MAX))
		dprintf(("Has size max"));

	if (virtio_host_supports(config, VIRTIO_BLK_F_FLUSH))
		dprintf(("Has flush"));

	if (virtio_host_supports(config, VIRTIO_BLK_F_BLK_SIZE)) {
		blk_config.blk_size = virtio_sread32(config, 20);
		dprintf(("Block Size: %d", blk_config.blk_size));
	}

	if (virtio_host_supports(config, VIRTIO_BLK_F_BARRIER))
		dprintf(("Has barrier"));

	return 0;
}

static int
virtio_blk_config(struct virtio_config *cfg, struct virtio_blk_config *blk_cfg)
{
	u32_t sectors_low, sectors_high, size_mbs;

	/* capacity is always there */
	sectors_low = virtio_sread32(cfg, 0);
	sectors_high = virtio_sread32(cfg, 4);
	blk_cfg->capacity = ((u64_t)sectors_high << 32) | sectors_low;


	/* If this gets truncated, you have a big disk... */
	size_mbs = (u32_t)(blk_cfg->capacity * 512 / 1024 / 1024);
	dprintf(("Capacity: %d MB", size_mbs));

	/* do feature setup */
	virtio_blk_feature_setup();

	virtio_blk_phys_mapping();

	return 0;
}

static int
virtio_blk_probe(void)
{
	config = virtio_setup_device(0x00002, name, 1, blkf,
				     sizeof(blkf) / sizeof(blkf[0]),
				     VIRTIO_BLK_NUM_THREADS, 0);
	if (config == NULL)
		return ENXIO;

	virtio_blk_config(config, &blk_config);

	virtio_irq_enable(config);

	return OK;
}

static int
sef_cb_init_fresh(int type, sef_init_info_t *UNUSED(info))
{
	if (virtio_blk_probe() != OK)
		panic("%s: No device found", name);

	blockdriver_announce(type);

	return OK;
}

static void
sef_cb_signal_handler(int signo)
{
	if (signo == SIGTERM) {
		terminating = 1;
		virtio_terminate();
	}
}

static void
sef_local_startup(void)
{
	sef_setcb_init_fresh(sef_cb_init_fresh);
	sef_setcb_init_lu(sef_cb_init_fresh);

	sef_setcb_signal_handler(sef_cb_signal_handler);

	sef_startup();
}

int
main(int argc, char **argv)
{
	env_setargs(argc, argv);
	sef_local_startup();
	blockdriver_mt_task(&virtio_blk_dtab);
	return OK;
}

/* 
 * Poor man's approach to virtio-blk
 */
#include <minix/drivers.h>
#include <minix/blockdriver_mt.h>
#include <minix/drvlib.h>
#include <sys/ioc_disk.h>
#include <sys/mman.h>

#include "virtio.h"
#include "virtio_ring.h"
#include "virtio_blk.h"

#include <assert.h>

/*
 * # mknod /dev/c2d0
 * # service up virtio_blk -dev /dev/c2d0 -devstyle STYLE_DEV
 */

/* From AHCI driver, currently ony V_INFO is used all the time */
enum {
	V_NONE,	/* no output at all; keep silent */
	V_ERR,	/* important error information only (the default) */
	V_INFO,	/* general information about the driver and devices */
	V_DEV,	/* device details, to help with detection problems */
	V_REQ	/* detailed information about requests */
};



#define dprintf(v,s) while (verbosity>= (v)) {		\
		printf("%s: ", config.name);		\
		printf s;				\
		printf("\n");				\
		break;	/* unconditional break */	\
}



struct virtio_feature blk_features[] = {
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


static unsigned int spurious_interrupts = 0;

static int open_count = 0;

static int verbosity = 4;		/* verbosity level (0..4) */

struct virtio_blk_config blk_config;

struct virtio_queue blk_queues[1];

struct virtio_config config = {
	"virtio-blk",
	-1,						/* io port */
	blk_features,
	sizeof(blk_features) / sizeof(blk_features[0]),
	blk_queues,
	sizeof(blk_queues) / sizeof(blk_queues[0]),
	-1,						/* irq */
	0,						/* irq hook */
	0,						/* msi */
	&blk_config,
	sizeof(blk_config),
};


#define MAX_DRIVES			1
#define SUB_PER_DRIVE			(NR_PARTITIONS * NR_PARTITIONS)
#define VIRTIO_BLK_SIZE			512

/* Actually, I think it "should" work */
#define NUM_THREADS			4

struct device part[DEV_PER_DRIVE];	/* partition */
struct device subpart[SUB_PER_DRIVE];	/* subpartitions */


/* Use for actual requests */
static struct virtio_blk_outhdr hdrs[NUM_THREADS];

/* So usually a status is only one byte, but we have
 * some alignment issues if we dont take 2 bytes.
 */
static u16_t status[NUM_THREADS];

/* Physical addresses */
static struct vumap_phys umap_hdrs[NUM_THREADS];
static struct vumap_phys umap_status[NUM_THREADS];

/* Prototypes */
static int virtio_open(dev_t minor, int access);
static int virtio_close(dev_t minor);
static ssize_t virtio_transfer(dev_t minor, int do_write, u64_t position,
	endpoint_t endpt, iovec_t *iovec, unsigned int count, int flags);
static int virtio_ioctl(dev_t minor, unsigned int request,
			endpoint_t endpt, cp_grant_id_t grant);
static struct device *virtio_part(dev_t minor);
static void virtio_geometry(dev_t minor, struct partition *entry);
static void virtio_intr(unsigned int UNUSED(irqs));
static int virtio_device(dev_t minor, device_id_t *id);


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

static int virtio_open(dev_t minor, int access)
{
	/* Read only devices should only be mounted... read-only */
	if ((access & W_BIT) && virtio_host_supports(&config, VIRTIO_BLK_F_RO))
		return EACCES;

	/* The device is opened for the first time.
	 *
	 * TODO:
	 * We could possibly defer allocating the queues to this
	 * point.
	 */
	if (open_count == 0) {
		part[0].dv_size = blk_config.capacity * VIRTIO_BLK_SIZE;
		partition(&virtio_blk_dtab, 0, P_PRIMARY, 0 /* ATAPI */);
		blockdriver_mt_set_workers(0, NUM_THREADS);
	}

	open_count++;
	return OK;
}

static int virtio_close(dev_t minor)
{
	if (open_count == 0) {
		dprintf(V_ERR, ("%s: One too many times...", config.name));
		return EINVAL;
	}

	open_count--;

	return OK;
}

static int check_addr_set_write_perm(iovec_s_t *iv, struct vumap_vir *vir,
				     struct vumap_phys *phys, int cnt,
				     int write)
{
	for (int i = 0; i < cnt ; i++) {

		/* So you gave us a byte aligned buffer? Good job! */
		if (phys[i].vp_addr & 1) {
			dprintf(V_ERR, ("odd buffer from %08lx",
					 phys[i+1].vp_addr));
			return EINVAL;
		}

		/* Check if the buffer is good */
		if (phys[i].vp_size != vir[i].vv_size) {
			dprintf(V_ERR, ("Non-contig buf %08lx",
					 phys[i+1].vp_addr));
			return EINVAL;
		}
		
		/* If write, the buffers only need to be read */
		phys[i].vp_addr |= !write;
	}

	return OK;
}

static int prepare_vir_vec(endpoint_t endpt, struct vumap_vir *vir,
			iovec_s_t *iv, int cnt, vir_bytes *size)
{
	/* This is pretty much the same as sum_iovec from AHCI,
	 * except that we don't support any iovecs where the size
	 * is not a multiple of 512
	 */
	vir_bytes tmp, total = 0;
	for (int i = 0; i < cnt; i++) {
		tmp = iv[i].iov_size;

		if (tmp == 0 || (tmp % VIRTIO_BLK_SIZE) || tmp > LONG_MAX) {
			dprintf(V_ERR, ("bad iv[%d].iov_size from %d", i, endpt));
			return EINVAL;
		}

		total += tmp;

		if (total > LONG_MAX) {
			dprintf(V_ERR, ("total overflow from %d", endpt));
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

static ssize_t virtio_transfer(dev_t minor, int write, u64_t position,
	endpoint_t endpt, iovec_t *iovec, unsigned int cnt, int flags)
{
	/* Need to translate vir to phys */
	struct vumap_vir vir[NR_IOREQS];

	/* Physical including header and trailer */
	struct vumap_phys phys[NR_IOREQS + 2];

	/* Which thread is doing this? */
	thread_id_t tid = blockdriver_mt_get_tid();

#if 0
	/* see whether we actually do multithreading */
	if (tid != 0) {
		dprintf(V_INFO, ("Surprise Surprise!!!"));
	}
#endif
	
	/* header and tailer */
	vir_bytes size = 0;
	struct device *dv;
	u64_t sector;
	u64_t end_part;
	int r, pcnt;

	iovec_s_t *iv = (iovec_s_t *)iovec;
	int access = write ? VUA_READ : VUA_WRITE;

	/* Don't touch this one anymore */
	iovec = NULL;
	
	if (cnt > NR_IOREQS)
		return EINVAL;

	/* position greater than capacity? */
	if (position >= blk_config.capacity * VIRTIO_BLK_SIZE)
		return OK;

	dv = virtio_part(minor);

	if (!dv)
		return ENXIO;


	position += dv->dv_base;
	end_part = dv->dv_base + dv->dv_size;

	/* Hmmm, AHCI tries to fix this up, but lets just say everything
	 * needs to be sector (512 byte ) aligned...
	 */
	if (position % VIRTIO_BLK_SIZE) {
		dprintf(V_ERR, ("Non sector-aligned access [%08x%08x]",
				(u32_t)(position >> 32), (u32_t)position));
		return EINVAL;
	}
	
	sector = position / VIRTIO_BLK_SIZE;

	r = prepare_vir_vec(endpt, vir, iv, cnt, &size);

	if (r != OK)
		return r;
	
	/* truncate if the partition is smaller than that */
	if (position + size > end_part - 1) {
		dprintf(V_INFO, ("Uhmm... dragons... everywhere..."));
		size = end_part - position;
	}

	if (size % VIRTIO_BLK_SIZE) {
		dprintf(V_ERR, ("non-sector sized (%lu) from %d",
				size, endpt))
		return EINVAL;
	}

	/* Map vir to phys */
	pcnt = cnt;
	if ((r = sys_vumap(endpt, vir, cnt, 0, access,
			   &phys[1], &pcnt)) != OK) {

		dprintf(V_ERR, ("Unable to map memory from %d (%d)",
				endpt, r));
		return r;
	}

#if 0	
	dprintf(V_INFO, ("transfer sector=%u size=%08x write=%d pcnt=%d",
			 (u32_t) sector, size, write, pcnt));
#endif

	/* First the header */
	phys[0].vp_addr = umap_hdrs[tid].vp_addr;
	phys[0].vp_size = sizeof(hdrs[0]);

	r = check_addr_set_write_perm(iv, vir, &phys[1], pcnt, write);

	if (r != OK)
		return r;

	/* Put the status at the end */
	assert(!(umap_status[tid].vp_addr & 1));
	phys[pcnt + 1].vp_addr = umap_status[tid].vp_addr;
	phys[pcnt + 1].vp_size = sizeof(u8_t);
	
	/* Status always needs write access */
	phys[1 + pcnt].vp_addr |= 1;


	/* prepare the header */
	if (write)
		hdrs[tid].type = VIRTIO_BLK_T_OUT;
	else
		hdrs[tid].type = VIRTIO_BLK_T_IN;

	hdrs[tid].ioprio = 0;
	hdrs[tid].sector = sector;


	/* Set status to "failure" */
	status[tid] = 0xFFFF;

	/* Go to the queue */
	virtio_to_queue(&config, 0, phys, 2 + pcnt, &tid);

	/* Wait for completion */
	blockdriver_mt_sleep();

	/* We only use the last 8 bits of status */
	if (status[tid] & 0xFF) {
		dprintf(V_ERR, ("ERR status=%02x", status[tid] & 0xFF));
		dprintf(V_ERR, ("ERR sector=%u size=%lx w=%d cnt=%d tid=%d",
			 	(u32_t) sector, size, write, pcnt, tid));

		for (int i = 0; i < pcnt + 2; i++) {
			dprintf(V_ERR, ("ERR phys[%02d] %08lx %u",
					i, phys[i].vp_addr, phys[i].vp_size));
		}

		return EIO;
	}
#if 0
	dprintf(V_INFO, ("transfer done=%u size=%08x write=%d pcnt=%d",
			 (u32_t) sector, size, write, pcnt));
#endif

	return size;
}



static int virtio_ioctl(dev_t minor, unsigned int request,
			endpoint_t endpt, cp_grant_id_t grant)
{
	switch (request) {

	case DIOCOPENCT:
		return sys_safecopyto(endpt, grant, 0,
			(vir_bytes) &open_count, sizeof(open_count));

	case DIOCFLUSH:
		if (!virtio_host_supports(&config, VIRTIO_BLK_F_FLUSH)) {
			/* But even if, we don't do flushes yet */
			return EIO;
		}
		
		dprintf(V_INFO, ("Host has no flush support"));

	}

	return EINVAL;
}


static struct device *virtio_part(dev_t minor)
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

static void virtio_geometry(dev_t minor, struct partition *entry)
{
	/* Only for the drive */
	if (minor != 0)
		return;

	/* Only if the host supports it */
	if(!virtio_host_supports(&config, VIRTIO_BLK_F_GEOMETRY))
		return;

	entry->cylinders = blk_config.geometry.cylinders;
	entry->heads = blk_config.geometry.heads;
	entry->sectors = blk_config.geometry.sectors;
}

static void virtio_intr(unsigned int UNUSED(irqs))
{
	thread_id_t *ret_tid;
	
	if (virtio_had_irq(&config)) {

		if (virtio_from_queue(&config, 0, (void**)&ret_tid))
			panic("Could not get data from queue");

		blockdriver_mt_wakeup(*ret_tid);
	} else {
		spurious_interrupts += 1;
#if 0
		dprintf(V_DEV, ("Spurious Interrupt %8u",
				spurious_interrupts));
#endif
	}
	virtio_irq_enable(&config);
}

static int virtio_device(dev_t minor, device_id_t *id)
{
	struct device *dev = virtio_part(minor);

	/* Check if this device exists */
	if (!dev)
		return ENXIO;

	*id = 0;
	return OK;
}

static void virtio_blk_phys_mapping(void)
{
	/* Hack to get the physical addresses of hdr and status */
	struct vumap_vir virs[NUM_THREADS];

	int r, cnt, pcnt;
	pcnt = cnt = NUM_THREADS;
	int access = VUA_READ  | VUA_WRITE;

	/* prepare array for hdr addresses */
	for (int i = 0; i < NUM_THREADS; i++) {
		virs[i].vv_addr = (vir_bytes)&hdrs[i];
		virs[i].vv_size = sizeof(hdrs[0]);
	}
	
	/* make the call */
	r = sys_vumap(SELF, virs, cnt, 0, access, umap_hdrs, &pcnt);

	if (r != OK)
		panic("%s: Unable to map hdrs %d", config.name, r);
	
	/* prepare array for status addresses */
	for (int i = 0; i < NUM_THREADS; i++) {
		assert(!((vir_bytes)(&status[i]) & 1));
		virs[i].vv_addr = (vir_bytes)&status[i];
		virs[i].vv_size = sizeof(status[0]);
	}

	pcnt = cnt = NUM_THREADS;
	
	r = sys_vumap(SELF, virs, cnt, 0, access, umap_status, &pcnt);
	
	if (r != OK)
		panic("%s: Unable to map status %d", config.name, r);
}

static int virtio_blk_feature_setup(void)
{
	/* Feature setup for virtio_blk
	 *
	 * FIXME: This only reads and prints stuff right now, not taking
	 * any into account.
	 *
	 * We use virtio_sread() here, to jump over the generic
	 * headers using magic numbers...
	 */
	if (virtio_host_supports(&config, VIRTIO_BLK_F_SEG_MAX)) {
		blk_config.seg_max = virtio_sread32(&config, 12);
		dprintf(V_INFO, ("Seg Max: %d", blk_config.seg_max));
	}
	
	if (virtio_host_supports(&config, VIRTIO_BLK_F_GEOMETRY)) {
		blk_config.geometry.cylinders = virtio_sread16(&config, 16);
		blk_config.geometry.heads= virtio_sread8(&config, 18);
		blk_config.geometry.sectors = virtio_sread8(&config, 19);
		dprintf(V_INFO, ("Geometry: cyl=%d heads=%d sectors=%d",
					blk_config.geometry.cylinders,
					blk_config.geometry.heads,
					blk_config.geometry.sectors));
	}
	
	if (virtio_host_supports(&config, VIRTIO_BLK_F_SIZE_MAX))
		dprintf(V_INFO, ("Has size max"));
	
	if (virtio_host_supports(&config, VIRTIO_BLK_F_FLUSH))
		dprintf(V_INFO, ("Has flush"));
	
	if (virtio_host_supports(&config, VIRTIO_BLK_F_BLK_SIZE)) {
		blk_config.blk_size = virtio_sread32(&config, 20);
		dprintf(V_INFO, ("Block Size: %d", blk_config.blk_size));
	}
	
	if (virtio_host_supports(&config, VIRTIO_BLK_F_BARRIER))
		dprintf(V_INFO, ("Has barrier"));

	return 0;
}

static int virtio_blk_config(struct virtio_config *cfg,
				  struct virtio_blk_config *blk_cfg)
{
	u32_t sectors_low, sectors_high, size_mbs;

	/* capacity is always there */
	sectors_low = virtio_sread32(cfg, 0);
	sectors_high = virtio_sread32(cfg, 4);
	blk_cfg->capacity = ((u64_t)sectors_high << 32) | sectors_low;


	/* If this gets truncated, you have a big disk... */
	size_mbs = (u32_t)(blk_cfg->capacity * 512 / 1024 / 1024);
	dprintf(V_INFO, ("Capacity: %d MB", size_mbs));

	/* do feature setup */
	virtio_blk_feature_setup();

	virtio_blk_phys_mapping();

	return 0;
}

static int virtio_blk_probe()
{
	dprintf(V_INFO, ("Looking up device..."));

	int devind = virtio_find_dev(0x0002);

	if (devind < 0) {
		dprintf(V_INFO, ("No device found :("));
		return -1;
	}

	dprintf(V_INFO, ("Found device at index=%d", devind));
	
	virtio_config(devind, &config);
	
	virtio_blk_config(&config, &blk_config);

	virtio_alloc_queues(&config);
	virtio_irq_register(&config);
	virtio_irq_enable(&config);

	return 0;
}

static int sef_cb_init_fresh(int type, sef_init_info_t *UNUSED(info))
{
	if (virtio_blk_probe())
		panic("no device found?");

	blockdriver_announce(type);

	return OK;
}

static void sef_cb_signal_handler(int signo)
{
	dprintf(V_INFO, ("Got signal"));
	
	dprintf(V_INFO, ("Terminating..."));
	virtio_reset(&config);
	virtio_irq_unregister(&config);
	exit(0);
}

static void sef_local_startup(void)
{
	sef_setcb_init_fresh(sef_cb_init_fresh);
	sef_setcb_init_lu(sef_cb_init_fresh);

	sef_setcb_signal_handler(sef_cb_signal_handler);

	sef_startup();
}

int main(int argc, char **argv)
{
	/* Driver task.
	 */
	env_setargs(argc, argv);
	sef_local_startup();

	blockdriver_mt_task(&virtio_blk_dtab);
	return OK;
}

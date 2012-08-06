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

/*
 * mknod /dev/c2d0
 * use service up virtio_blk -dev /dev/c2d0 -devstyle STYLE_DEV
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
	-1,						/* port */
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


/* So, we only have a single drive here and need to go through all
 * this trouble?
 *
 * DEV_PER_DRIVE is NR_PARTITIONS + 1
 * and NR_PARTITIONS is 4.
 *
 * SUB_PER_DRIVE is defined by ahci as NR_PARTITIONS^2
 * --> 16
 * c0 c0d0 c0d1 c0d2 c0d3
 * ^^^^^^^^^^^^^^^^^^^^^^
 *    5 minors?
 *
 * p0s0 p0s1 p0s2 p0s3
 * ^^^^^^^^^^^^^^^^^^^
 *    for c0dx for x > 0 --> 16
 *
 * Is this true? And why should we actually care?
 *
 */
#define MAX_DRIVES			1
#define SUB_PER_DRIVE			(NR_PARTITIONS * NR_PARTITIONS)
#define NR_SUBDEVS			(MAX_DRIVES * SUB_PER_DRIVE)
#define NR_MINORS			(MAX_DRIVES * DEV_PER_DRIVE)

#define VIRTIO_BLK_SIZE			512

/* Actually, I think it "should" work */
#define NUM_THREADS			4

struct device part[DEV_PER_DRIVE];	/* partition */
struct device subpart[SUB_PER_DRIVE];	/* subpartitions */


/* Use for actual requests */
static struct virtio_blk_outhdr hdrs[NUM_THREADS];
static u8_t status[NUM_THREADS];

/* Physical addresses */
static phys_bytes hdr_paddrs[NUM_THREADS];
static u8_t status_paddrs[NUM_THREADS];

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
	if ((access & W_BIT) &&
		virtio_host_supports(&config, VIRTIO_BLK_F_RO))
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
	return 0;
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

static ssize_t virtio_transfer(dev_t minor, int write, u64_t position,
	endpoint_t endpt, iovec_t *iovec, unsigned int cnt, int flags)
{
	/* need to translate vir to phys */
	struct vumap_vir vir[NR_IOREQS];
	struct vumap_phys phys[NR_IOREQS];
	thread_id_t tid = blockdriver_mt_get_tid();

	if (tid != 0)
		dprintf(V_INFO, ("Surprise Surprise!!!"));
	
	/* header and tailer */
	struct virtio_buf_desc vbufs[NR_IOREQS + 2];
	size_t size = 0;
	struct device *dv;
	u64_t sector;
	u64_t end_part;
	iovec_s_t *iovec_s;
	int i, r, access, pcnt = NR_IOREQS;

	
	if (cnt > NR_IOREQS)
		return EINVAL;

	/* position greater than capacity? */
	if (position >= blk_config.capacity * VIRTIO_BLK_SIZE)
		return OK;

	dv = virtio_part(minor);
	position += dv->dv_base;
	end_part = dv->dv_base + dv->dv_size;

	/* Hmmm, AHCI tries to fix this up, but lets just say everything
	 * needs to be sector aligned...
	 */
	if (position % VIRTIO_BLK_SIZE) {
		dprintf(V_ERR, ("No no no :( [%08x]", (u32_t)position));
		return EINVAL;
	}
	
	sector = position / VIRTIO_BLK_SIZE;

	iovec_s = (iovec_s_t *)iovec;
	
	/* AHCI does a lot of checks here... let's just go ahead
	 * and trust the caller... ...cough...
	 *
	 * Also, we actually "only" need the physical address.
	 * Any faster way to get them?
	 */
	for (i = 0; i < cnt; i++) {
		if (endpt == SELF)
			vir[i].vv_addr = (vir_bytes) iovec_s[i].iov_grant;
		else
			vir[i].vv_grant = iovec_s[i].iov_grant;
		vir[i].vv_size = iovec_s[i].iov_size;

		size += iovec_s[i].iov_size;
	}
	pcnt = cnt;
	
	/* truncate if the partition is smaller than that */
	if (position + size > end_part - 1) {
		dprintf(V_INFO, ("Uhmm... dragons... everywhere..."));
		size = end_part - position;
	}

	/* If we write, we only need to read the bufs, but.... */
	access = write ? VUA_READ : VUA_WRITE;

	/* Ok map everything to phys please */
	if ((r = sys_vumap(endpt, vir, cnt, 0, access, phys, &pcnt)) != OK) {
		dprintf(V_ERR, ("Unable to map memory from %d (%d)",
				endpt, r));
		return r;
	}
#if 0	
	dprintf(V_INFO, ("transfer sector=%u size=%08x write=%d pcnt=%d",
			 (u32_t) sector, size, write, pcnt));
#endif

	vbufs[0].phys = hdr_paddrs[tid];
	vbufs[0].len = sizeof(hdrs[0]);
	vbufs[0].write = 0;
	
	for (i = 0; i < pcnt; i++) {
		vbufs[i + 1].phys = phys[i].vp_addr;
		vbufs[i + 1].len = iovec_s[i].iov_size;
		/* if we write, we don't write into the buffer */
		vbufs[i + 1].write = !write;
	}

	vbufs[i + 1].phys = status_paddrs[tid];
	vbufs[i + 1].len = sizeof(status);
	vbufs[i + 1].write = 1;

	if (write)
		hdrs[tid].type = VIRTIO_BLK_T_OUT;
	else
		hdrs[tid].type = VIRTIO_BLK_T_IN;

	hdrs[tid].ioprio = 0;
	hdrs[tid].sector = sector;

	status[tid] = 0;
	/* Uhm... tid is on the statck, hope that's fine */
	virtio_to_queue(&config, 0, vbufs, 2 + pcnt, &tid);

	/* wait for completion */
	blockdriver_mt_sleep();

	if (status[tid]) {
		dprintf(V_ERR, ("status=%02x", status[tid]));
		dprintf(V_ERR, ("ER sector=%u size=%x w=%d cnt=%d tid=%d",
			 	(u32_t) sector, size, write, pcnt, tid));
		return EIO;
	}

	return size;
}



static int virtio_ioctl(dev_t minor, unsigned int request,
			endpoint_t endpt, cp_grant_id_t grant)
{
	switch (request) {

	case DIOCOPENCT:
		return sys_safecopyto(endpt, grant, 0,
			(vir_bytes) open_count, sizeof(open_count));

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
	/* So this whole partition thing... I don't trust... */
	
	if (minor < NR_MINORS) {
		return &part[minor % DEV_PER_DRIVE];
	} else {
		if ((unsigned)(minor -= MINOR_d0p0s0) < NR_SUBDEVS) {
			return &subpart[minor % SUB_PER_DRIVE];
		}
	}

	panic("No surprise, virtio_part() is broken....");
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
		if(virtio_from_queue(&config, 0, (void**)&ret_tid))
			panic("Could not get data from queue");
		blockdriver_mt_wakeup(*ret_tid);
	} else {
		spurious_interrupts += 1;
		dprintf(V_DEV, ("Spurious Interrupt %8u",
				spurious_interrupts));
	}
	virtio_irq_enable(&config);
}

static int virtio_device(dev_t minor, device_id_t *id)
{
	/* we only have a single drive here */
	*id = 0;
	return 0;
}

static void virtio_blk_phys_mapping(void)
{
	/* Hack to get the physical addresses of hdr and status */
	struct vumap_vir virs[NUM_THREADS * 2];
	struct vumap_phys phys[NUM_THREADS * 2];

	int r, pcnt, cnt;
	pcnt = cnt = 2 * NUM_THREADS;
	int access = VUA_READ  | VUA_WRITE;

	/* prepare array */
	for (int i = 0; i < 2 * NUM_THREADS; i += 2) {
		virs[i].vv_addr = (vir_bytes)&hdrs[i];
		virs[i].vv_size = sizeof(hdrs[0]);
		virs[i + 1].vv_addr = (vir_bytes)&status[i];
		virs[i + 1].vv_size = sizeof(status[0]);
	}

	/* make the call */
	if ((r = sys_vumap(SELF, virs, cnt, 0, access, phys, &pcnt))) {
		panic("%s: Unable to map hdrs and status", config.name);
	}
	
	/* store result */
	for (int i = 0; i < NUM_THREADS; i++) {
		hdr_paddrs[i] = phys[i].vp_addr;
		status_paddrs[i] = phys[i + 1].vp_addr;
	}
}

static int virtio_blk_fsetup(void)
{
	/* Feature setup for virtio_blk
	 *
	 * FIXME: This only reads and prints stuff right now, not taking
	 * any into account.
	 *
	 * We use virtio_sread() here, to jump over the generic
	 * headers using great magic number...
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
	virtio_blk_fsetup();

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
	return 0;
}

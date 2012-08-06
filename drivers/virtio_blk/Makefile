# Makefile for the Advanced Host Controller Interface driver (AHCI)
PROG=	virtio_blk
SRCS=	virtio_blk.c virtio.c

DPADD+=	${LIBBLOCKDRIVER} ${LIBSYS} ${LIBTIMERS} ${LIBMTHREAD}
LDADD+=	-lblockdriver -lsys -ltimers -lmthread

MAN=

BINDIR?= /sbin

.include <minix.service.mk>

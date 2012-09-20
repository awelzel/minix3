# Makefile for virtio_blk device
PROG=	virtio_blk
SRCS=	virtio_blk.c

DPADD+=	${LIBBLOCKDRIVER} ${LIBSYS} ${LIBTIMERS} ${LIBMTHREAD} ${LIBVIRTIO}
LDADD+=	-lvirtio -lblockdriver -lsys -ltimers -lmthread

MAN=

BINDIR?= /sbin

.include <minix.service.mk>

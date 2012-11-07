#
# Makefile for virtio_net
#
PROG=	virtio_net
SRCS=	virtio_net.c

FILES=$(PROG).conf
FILESNAME=$(PROG)
FILESDIR= /etc/system.conf.d

DPADD+=	${LIBNETDRIVER} ${LIBSYS} ${LIBTIMERS} ${LIBVIRTIO}
LDADD+=	-lvirtio -lnetdriver -lsys -ltimers

MAN=

BINDIR?= /usr/sbin

.include <minix.service.mk>

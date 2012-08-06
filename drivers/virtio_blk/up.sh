#!/bin/sh
service run `pwd`/virtio_blk -dev /dev/c2d0 -devstyle STYLE_DEV

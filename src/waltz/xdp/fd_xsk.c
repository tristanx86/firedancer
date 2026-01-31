#if !defined(__linux__)
#error "fd_xsk requires Linux operating system with XDP support"
#endif

#define _GNU_SOURCE /* MADV_DONTDUMP */

#include <errno.h>
#include <stdio.h> /* snprintf */
#include <unistd.h>
#include <sys/mman.h> /* mmap */
#include <sys/types.h>
#include <sys/socket.h> /* sendto */
#include <sys/syscall.h> /* SYS_mlock */
#include <sys/ioctl.h> /* ioctl, EPIOCSPARAMS */
#include <fcntl.h>
#include "../../util/log/fd_log.h"
#include "fd_xsk.h"

/* Support for older kernels */
 
#ifndef EPIOCSPARAMS
#define EPIOCSPARAMS _IOW('E', 0x02, fd_epoll_params_t)
#endif

#ifndef SO_BUSY_POLL
#define SO_BUSY_POLL 46
#endif

#ifndef SO_INCOMING_NAPI_ID
#define SO_INCOMING_NAPI_ID	56
#endif

#ifndef SO_PREFER_BUSY_POLL
#define SO_PREFER_BUSY_POLL	69
#endif

#ifndef SO_BUSY_POLL_BUDGET
#define SO_BUSY_POLL_BUDGET	70
#endif

/* Join/leave *********************************************************/

/* fd_xsk_mmap_offset_cstr: Returns a cstr describing the given offset
   param (6th argument of mmap(2)) assuming fd (5th param of mmap(2)) is
   an XSK file descriptor.  Returned cstr is valid until next call. */
static char const *
fd_xsk_mmap_offset_cstr( long mmap_off ) {
  switch( mmap_off ) {
  case XDP_PGOFF_RX_RING:              return "XDP_PGOFF_RX_RING";
  case XDP_PGOFF_TX_RING:              return "XDP_PGOFF_TX_RING";
  case XDP_UMEM_PGOFF_FILL_RING:       return "XDP_UMEM_PGOFF_FILL_RING";
  case XDP_UMEM_PGOFF_COMPLETION_RING: return "XDP_UMEM_PGOFF_COMPLETION_RING";
  default: {
    static char buf[ 19UL ];
    snprintf( buf, 19UL, "0x%lx", (ulong)mmap_off );
    return buf;
  }
  }
}

/* fd_xsk_mmap_ring maps the given XSK ring into the local address space
   and populates fd_ring_desc_t.  Every successful call to this function
   should eventually be paired with a call to fd_xsk_munmap_ring(). */
static int
fd_xsk_mmap_ring( fd_xdp_ring_t * ring,
                  int             xsk_fd,
                  long            map_off,
                  ulong           elem_sz,
                  ulong           depth,
                  struct xdp_ring_offset const * ring_offset ) {
  /* TODO what is ring_offset->desc ? */

  /* sanity check */
  if( depth > (ulong)UINT_MAX ) {
    return -1;
  }

  ulong map_sz = ring_offset->desc + depth*elem_sz;

  void * res = mmap( NULL, map_sz, PROT_READ|PROT_WRITE, MAP_SHARED, xsk_fd, map_off );
  if( FD_UNLIKELY( res==MAP_FAILED ) ) {
    FD_LOG_WARNING(( "mmap(NULL, %lu, PROT_READ|PROT_WRITE, MAP_SHARED, xsk_fd, %s) failed (%i-%s)",
                     map_sz, fd_xsk_mmap_offset_cstr( map_off ), errno, fd_io_strerror( errno ) ));
    return -1;
  }

  /* Lock descriptor rings to prevent swapping. Also advise the
     kernel to exclude this region from core dumps for consistency
     with fd_shmem. Reimplements syscall logic of fd_numa_mlock()
     from fd_shmem_private.h to circumvent the ASan interceptor
     and avoid private header dependencies. */

  if( FD_UNLIKELY( (int)syscall( SYS_mlock, res, map_sz ) ) )
    FD_LOG_WARNING(( "syscall(SYS_mlock, %p, %lu KiB) on %s ring failed (%i-%s); attempting to continue",
                     res, map_sz>>10, fd_xsk_mmap_offset_cstr( map_off ), errno, fd_io_strerror( errno ) ));

  if( FD_UNLIKELY( madvise( res, map_sz, MADV_DONTDUMP ) ) )
    FD_LOG_WARNING(( "madvise(%p, %lu KiB) on %s ring failed (%i-%s); attempting to continue",
                     res, map_sz>>10, fd_xsk_mmap_offset_cstr( map_off ), errno, fd_io_strerror( errno ) ));

  /* TODO add unit test asserting that cached prod/cons seq gets
          cleared on join */
  fd_memset( ring, 0, sizeof(fd_xdp_ring_t) );

  ring->mem    = res;
  ring->map_sz = map_sz;
  ring->depth  = (uint)depth;
  ring->ptr    = (void *)( (ulong)res + ring_offset->desc     );
  ring->flags  = (uint *)( (ulong)res + ring_offset->flags    );
  ring->prod   = (uint *)( (ulong)res + ring_offset->producer );
  ring->cons   = (uint *)( (ulong)res + ring_offset->consumer );

  return 0;
}

/* fd_xsk_munmap_ring unmaps the given XSK ring from the local address
   space and zeroes fd_ring_desc_t. */
static void
fd_xsk_munmap_ring( fd_xdp_ring_t * ring,
                    long             map_off ) {
  if( FD_UNLIKELY( !ring->mem ) ) return;

  void * mem = ring->mem;
  ulong  sz  = ring->map_sz;

  fd_memset( ring, 0, sizeof(fd_xdp_ring_t) );

  if( FD_UNLIKELY( 0!=munmap( mem, sz ) ) )
    FD_LOG_WARNING(( "munmap(%p, %lu) on %s ring failed (%i-%s)",
                     mem, sz, fd_xsk_mmap_offset_cstr( map_off ), errno, fd_io_strerror( errno ) ));
}

/* fd_xsk_cleanup undoes a (partial) join by releasing all active kernel
   objects, such as mapped memory regions and file descriptors.  Assumes
   that no join to `xsk` is currently being used. */

fd_xsk_t *
fd_xsk_fini( fd_xsk_t * xsk ) {
  /* Undo memory mappings */

  fd_xsk_munmap_ring( &xsk->ring_rx, XDP_PGOFF_RX_RING              );
  fd_xsk_munmap_ring( &xsk->ring_tx, XDP_PGOFF_TX_RING              );
  fd_xsk_munmap_ring( &xsk->ring_fr, XDP_UMEM_PGOFF_FILL_RING       );
  fd_xsk_munmap_ring( &xsk->ring_cr, XDP_UMEM_PGOFF_COMPLETION_RING );

  /* Release XSK */

  if( FD_LIKELY( xsk->xsk_fd>=0 ) ) {
    /* Clear XSK descriptors */
    fd_memset( &xsk->offsets, 0, sizeof(struct xdp_mmap_offsets) );
    /* Close XSK */
    close( xsk->xsk_fd );
    xsk->xsk_fd = -1;
  }

  /* Release epoll_event struct  */

  return xsk;
}

/* fd_xsk_setup_umem: Initializes xdp_umem_reg and hooks up XSK with
   UMEM rings via setsockopt(). Retrieves xdp_mmap_offsets via
   getsockopt().  Returns 0 on success, -1 on failure. */
static int
fd_xsk_setup_umem( fd_xsk_t *              xsk,
                   fd_xsk_params_t const * params ) {

  /* Initialize xdp_umem_reg */
  struct xdp_umem_reg umem_reg = {
    .addr       = (ulong)params->umem_addr,
    .len        = params->umem_sz,
    .chunk_size = (uint)params->frame_sz,
  };

  /* Register UMEM region */
  int res;
  res = setsockopt( xsk->xsk_fd, SOL_XDP, XDP_UMEM_REG,
                    &umem_reg, sizeof(struct xdp_umem_reg) );
  if( FD_UNLIKELY( res!=0 ) ) {
    FD_LOG_WARNING(( "setsockopt(SOL_XDP,XDP_UMEM_REG(addr=%p,len=%lu,chunk_size=%lu)) failed (%i-%s)",
                     (void *)umem_reg.addr, (ulong)umem_reg.len, (ulong)umem_reg.chunk_size,
                     errno, fd_io_strerror( errno ) ));
    return -1;
  }

  /* Set ring frame counts */
# define FD_SET_XSK_RING_DEPTH(name, var)                                 \
    do {                                                                  \
      res = setsockopt( xsk->xsk_fd, SOL_XDP, name, &(var), 8UL );        \
      if( FD_UNLIKELY( res!=0 ) ) {                                       \
        FD_LOG_WARNING(( "setsockopt(SOL_XDP," #name ",%lu) failed (%i-%s)", \
                         var, errno, fd_io_strerror( errno ) ));          \
        return -1;                                                        \
      }                                                                   \
    } while(0)
  FD_SET_XSK_RING_DEPTH( XDP_UMEM_FILL_RING,       params->fr_depth );
  FD_SET_XSK_RING_DEPTH( XDP_RX_RING,              params->rx_depth );
  FD_SET_XSK_RING_DEPTH( XDP_TX_RING,              params->tx_depth );
  FD_SET_XSK_RING_DEPTH( XDP_UMEM_COMPLETION_RING, params->cr_depth );
# undef FD_SET_XSK_RING_DEPTH

  /* Request ring offsets */
  socklen_t offsets_sz = sizeof(struct xdp_mmap_offsets);
  res = getsockopt( xsk->xsk_fd, SOL_XDP, XDP_MMAP_OFFSETS,
                    &xsk->offsets, &offsets_sz );
  if( FD_UNLIKELY( res!=0 ) ) {
    FD_LOG_WARNING(( "getsockopt(SOL_XDP, XDP_MMAP_OFFSETS) failed (%i-%s)", errno, fd_io_strerror( errno ) ));
    return -1;
  }

  /* OK */
  return 0;
}

/* fd_xsk_setup_napi: Set irq-suspend-timeout, napi-defer-hard-irqs,
   gro-flush-timeout for the socket's associated napi queue using
   the netdev-genl linux api. */

static int
fd_xsk_setup_napi( fd_xsk_t *              xsk,
                   fd_xsk_params_t const * params ) {
    /* TODO: Finish */
    (void)xsk;
    (void)params;

    return 0;
}

/* fd_xsk_setup_poll: Setup preferred busy polling if the user has
   set that to be their preferred polling method */

static void
fd_xsk_setup_poll( fd_xsk_t *              xsk,
                   fd_xsk_params_t const * params ) {
    xsk->prefbusy_poll_enabled = 0;
    if( 0!=strcmp( params->poll_mode, "prefbusy" ) ) return;

    /* Configure socket options for preferred busy polling */

    int prefbusy_poll = 1;
    if( FD_UNLIKELY( 0!=setsockopt( xsk->xsk_fd, SOL_SOCKET, SO_PREFER_BUSY_POLL, &prefbusy_poll, sizeof(int) ) ) ) {
        int err = errno;
        FD_LOG_WARNING(( "setsockopt(xsk_fd,SOL_SOCKET,SO_PREFER_BUSY_POLL,1) failed (%i-%s)", err, fd_io_strerror( err ) ));
        if( err==EINVAL ) {
            FD_LOG_WARNING(( "Hint: Does your kernel support preferred busy polling? SO_PREFER_BUSY_POLL is available since Linux 5.11" ));
        }
        return;
    }

    if( FD_UNLIKELY( 0!=setsockopt( xsk->xsk_fd, SOL_SOCKET, SO_BUSY_POLL, &params->busy_poll_usecs, sizeof(uint) ) ) ) {
      FD_LOG_WARNING(( "setsockopt(xsk_fd,SOL_SOCKET,SO_BUSY_POLL,%u) failed (%i-%s)",
                       params->busy_poll_usecs, errno, fd_io_strerror( errno ) ));
      return;
    }

    /* Max busy_poll_budget can be is 64 */
    uint busy_poll_budget = 64U;
    if( FD_UNLIKELY( 0!=setsockopt( xsk->xsk_fd, SOL_SOCKET, SO_BUSY_POLL_BUDGET, &busy_poll_budget, sizeof(uint) ) ) ) {
      FD_LOG_WARNING(( "setsockopt(xsk_fd,SOL_SOCKET,SO_BUSY_POLL_BUDGET,%u) failed (%i-%s)",
                       busy_poll_budget, errno, fd_io_strerror( errno ) ));
      return;
    }

    /* Set socket non blocking */

    int sk_flags = fcntl( xsk->xsk_fd, F_GETFL, 0 );
    if( FD_UNLIKELY( sk_flags == -1 ) ) {
        FD_LOG_WARNING(( "fcntl(xsk->xsf_fd, F_GETFL, 0) failed (%i-%s)",
                         errno, fd_io_strerror( errno ) ));
        return;
    }
    if( FD_UNLIKELY( fcntl( xsk->xsk_fd, F_SETFL, sk_flags | O_NONBLOCK ) ) == -1 ) {
        FD_LOG_WARNING(( "fcntl(xsk->xsk_fd, F_SETFL, sk_flags | O_NONBLOCK) failed (%i-%s)",
                          errno, fd_io_strerror( errno ) ));
        return;
    }

    /* Create epoll instance */

    xsk->epoll_fd = epoll_create1(EPOLL_CLOEXEC);

    if( FD_UNLIKELY( xsk->epoll_fd < 0 ) ) {
        FD_LOG_WARNING(( "epoll_create1(EPOLL_CLOEXEC) failed (%i-%s)",
                         errno, fd_io_strerror( errno ) ));
        return;
    }

    /* Configure epoll instance event settings for edge triggered mode */

    fd_epoll_event_t event_param;
    event_param.events   = EPOLLIN | EPOLLET;
    event_param.data.ptr = NULL; /* NULL to mean the listening socket */
    if( FD_UNLIKELY( 0!=epoll_ctl( xsk->epoll_fd, EPOLL_CTL_ADD, xsk->xsk_fd, &event_param ) ) ) {
        FD_LOG_WARNING(( "epoll_ctl(xsk->epoll_fd, EPOLL_CTL_ADD, xsk->xsk_fd, &event_param) failed (%i-%s)",
                         errno, fd_io_strerror( errno ) ));
        close( xsk->epoll_fd );
        return;
    }

    /* Configure epoll instance parameters for prefbusy polling.
       Without this epoll is just a spectator informing userspace
       about events and not telling napi to poll the NIC driver. */

    fd_epoll_params_t epoll_params;
    epoll_params.busy_poll_usecs  = params->busy_poll_usecs;
    epoll_params.busy_poll_budget = (ushort)busy_poll_budget;
    epoll_params.prefer_busy_poll = 1U;
    if( FD_UNLIKELY( 0!=ioctl( xsk->epoll_fd, EPIOCSPARAMS, &epoll_params ) ) ) {
        FD_LOG_WARNING(( "ioctl(xsk->epoll_fd, EPIOCSPARAMS, &epoll_params) failed (%i-%s), switching to less performant fallback softirq based polling, likely due to being on a linux kernel older than v6.13.",
                         errno, fd_io_strerror( errno ) ));
        close( xsk->epoll_fd );
        return;
    }

    /* Configure napi with netdev if netdev is available (checked
       by sysfs-poll.c) and the user has netdev based configuration
       for preferred busy polling enabled */

    if( FD_UNLIKELY( 0!=fd_xsk_setup_napi( xsk, params ) ) ) {
        FD_LOG_WARNING(( "fd_xsk_setup_napi(xsk) failed" ));
        close( xsk->epoll_fd );
        return;
    }

    /* Successfully finished setting up prefbusy polling */
    xsk->prefbusy_poll_enabled = 1U;
}

/* fd_xsk_init: Creates and configures an XSK socket object, and
   attaches to a preinstalled XDP program.  The various steps are
   implemented in fd_xsk_setup_{...}. */

fd_xsk_t *
fd_xsk_init( fd_xsk_t *              xsk,
             fd_xsk_params_t const * params ) {

  if( FD_UNLIKELY( !xsk ) ) { FD_LOG_WARNING(( "NULL xsk" )); return NULL; }
  memset( xsk, 0, sizeof(fd_xsk_t) );

  if( FD_UNLIKELY( !params->if_idx ) ) { FD_LOG_WARNING(( "zero if_idx" )); return NULL; }
  if( FD_UNLIKELY( (!params->fr_depth) | (!params->rx_depth) |
                   (!params->tx_depth) | (!params->cr_depth) ) ) {
    FD_LOG_WARNING(( "invalid {fr,rx,tx,cr}_depth" ));
    return NULL;
  }
  if( FD_UNLIKELY( !params->umem_addr ) ) {
    FD_LOG_WARNING(( "NULL umem_addr" ));
    return NULL;
  }
  if( FD_UNLIKELY( !fd_ulong_is_aligned( (ulong)params->umem_addr, 4096UL ) ) ) {
    FD_LOG_WARNING(( "misaligned params->umem_addr" ));
    return NULL;
  }
  if( FD_UNLIKELY( !params->frame_sz || !fd_ulong_is_pow2( params->frame_sz ) ) ) {
    FD_LOG_WARNING(( "invalid frame_sz" ));
    return NULL;
  }

  xsk->if_idx      = params->if_idx;
  xsk->if_queue_id = params->if_queue_id;

  /* Create XDP socket (XSK) */

  xsk->xsk_fd = socket( AF_XDP, SOCK_RAW, 0 );
  if( FD_UNLIKELY( xsk->xsk_fd<0 ) ) {
    FD_LOG_WARNING(( "Failed to create XSK (%i-%s)", errno, fd_io_strerror( errno ) ));
    return NULL;
  }

  /* Associate UMEM region of fd_xsk_t with XSK via setsockopt() */

  if( FD_UNLIKELY( 0!=fd_xsk_setup_umem( xsk, params ) ) ) goto fail;

  /* Map XSK rings into local address space */

  if( FD_UNLIKELY( 0!=fd_xsk_mmap_ring( &xsk->ring_rx, xsk->xsk_fd, XDP_PGOFF_RX_RING,              sizeof(struct xdp_desc), params->rx_depth, &xsk->offsets.rx ) ) ) goto fail;
  if( FD_UNLIKELY( 0!=fd_xsk_mmap_ring( &xsk->ring_tx, xsk->xsk_fd, XDP_PGOFF_TX_RING,              sizeof(struct xdp_desc), params->tx_depth, &xsk->offsets.tx ) ) ) goto fail;
  if( FD_UNLIKELY( 0!=fd_xsk_mmap_ring( &xsk->ring_fr, xsk->xsk_fd, XDP_UMEM_PGOFF_FILL_RING,       sizeof(ulong),           params->fr_depth, &xsk->offsets.fr ) ) ) goto fail;
  if( FD_UNLIKELY( 0!=fd_xsk_mmap_ring( &xsk->ring_cr, xsk->xsk_fd, XDP_UMEM_PGOFF_COMPLETION_RING, sizeof(ulong),           params->cr_depth, &xsk->offsets.cr ) ) ) goto fail;

  /* Bind XSK to queue on network interface */

  uint flags = XDP_USE_NEED_WAKEUP | params->bind_flags;
  struct sockaddr_xdp sa = {
    .sxdp_family   = PF_XDP,
    .sxdp_ifindex  = xsk->if_idx,
    .sxdp_queue_id = xsk->if_queue_id,
    /* See extended commentary below for details on XDP_USE_NEED_WAKEUP
       flag. */
    .sxdp_flags    = (ushort)flags
  };

  char if_name[ IF_NAMESIZE ] = {0};

  if( FD_UNLIKELY( 0!=bind( xsk->xsk_fd, (void *)&sa, sizeof(struct sockaddr_xdp) ) ) ) {
    FD_LOG_WARNING(( "bind( PF_XDP, ifindex=%u (%s), queue_id=%u, flags=%x ) failed (%i-%s)",
                     xsk->if_idx, if_indextoname( xsk->if_idx, if_name ),
                     xsk->if_queue_id, flags,
                     errno, fd_io_strerror( errno ) ));
    goto fail;
  }

  /* We've seen that some popular Intel NICs seem to have a bug that
     prevents them from working in SKB mode with certain kernel
     versions.  We can identify them by sendto returning ENXIO or EINVAL
     in newer versions.  The core of the problem is that the kernel
     calls the generic ndo_bpf pointer instead of the driver-specific
     version.  This means that the driver's pointer to the BPF program
     never gets set, yet the driver's wakeup function gets called. */
  if( FD_UNLIKELY( -1==sendto( xsk->xsk_fd, NULL, 0, MSG_DONTWAIT, NULL, 0 ) ) ) {
    if( FD_LIKELY( errno==ENXIO || errno==EINVAL ) ) {
      FD_LOG_ERR(( "xsk sendto failed xsk_fd=%d (%i-%s).  This likely indicates "
                   "a bug with your NIC driver.  Try switching XDP mode using "
                   "net.xdp.xdp_mode in the configuration TOML.\n"
                   "Certain Intel NICs with certain driver/kernel combinations "
                   "are known to exhibit this issue in skb mode but work in drv "
                   "mode.", xsk->xsk_fd, errno, fd_io_strerror( errno ) ));
    } else {
      FD_LOG_WARNING(( "xsk sendto failed xsk_fd=%d (%i-%s)", xsk->xsk_fd, errno, fd_io_strerror( errno ) ));
    }
  }

  /* XSK successfully configured.  Traffic will arrive in XSK after
     configuring an XDP program to forward packets via XDP_REDIRECT.
     This requires providing the XSK file descriptor to the program via
     XSKMAP and is done in a separate step. */

  FD_LOG_INFO(( "AF_XDP socket initialized: bind( PF_XDP, ifindex=%u (%s), queue_id=%u, flags=%x ) success",
                xsk->if_idx, if_indextoname( xsk->if_idx, if_name ), xsk->if_queue_id, flags ));

  /* Check if the XSK is aware of the driver's NAPI ID for the
     associated RX queue.  Without it, preferred busy polling is not
     going to work correctly. */

  socklen_t napi_id_sz = sizeof(uint);
  if( FD_UNLIKELY( 0!=getsockopt( xsk->xsk_fd, SOL_SOCKET, SO_INCOMING_NAPI_ID, &xsk->napi_id, &napi_id_sz ) ) ) {
    if( errno==ENOPROTOOPT ) {
      xsk->napi_id = 0;
    } else {
      FD_LOG_WARNING(( "getsockopt(SOL_SOCKET,SO_INCOMING_NAPI_ID) failed (%i-%s)", errno, fd_io_strerror( errno ) ));
      goto fail;
    }
  }

  if( xsk->napi_id ) {
    FD_LOG_DEBUG(( "Interface %u Queue %u has NAPI ID %u", xsk->if_idx, xsk->if_queue_id, xsk->napi_id ));
  } else {
    FD_LOG_DEBUG(( "Interface %u Queue %u has unknown NAPI ID", xsk->if_idx, xsk->if_queue_id ));
  }

  /* If requested, enable preferred busy polling */

  fd_xsk_setup_poll( xsk, params );

  return xsk;

fail:
  fd_xsk_fini( xsk );
  return NULL;
}

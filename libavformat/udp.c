/*
 * UDP prototype streaming system
 * Copyright (c) 2000, 2001, 2002 Fabrice Bellard
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * UDP protocol
 */

#define _DEFAULT_SOURCE
#define _BSD_SOURCE     /* Needed for using struct ip_mreq with recent glibc */

#include "avformat.h"
#include "avio_internal.h"
#include "libavutil/avassert.h"
#include "libavutil/parseutils.h"
#include "libavutil/fifo.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/avstring.h"
#include "libavutil/opt.h"
#include "libavutil/log.h"
#include "libavutil/time.h"
#include "internal.h"
#include "network.h"
#include "os_support.h"
#include "url.h"
#include "ip.h"
#include "longestpath.h"

#ifdef __APPLE__
#include "TargetConditionals.h"
#endif

#if HAVE_UDPLITE_H
#include "udplite.h"
#else
/* On many Linux systems, udplite.h is missing but the kernel supports UDP-Lite.
 * So, we provide a fallback here.
 */
#define UDPLITE_SEND_CSCOV                               10
#define UDPLITE_RECV_CSCOV                               11
#endif

#ifndef IPPROTO_UDPLITE
#define IPPROTO_UDPLITE                                  136
#endif

#if HAVE_W32THREADS
#undef HAVE_PTHREAD_CANCEL
#define HAVE_PTHREAD_CANCEL 1
#endif

#if HAVE_PTHREAD_CANCEL
#include "libavutil/thread.h"
#endif

#ifndef IPV6_ADD_MEMBERSHIP
#define IPV6_ADD_MEMBERSHIP IPV6_JOIN_GROUP
#define IPV6_DROP_MEMBERSHIP IPV6_LEAVE_GROUP
#endif

#define UDP_TX_BUF_SIZE 32768
#define UDP_RX_BUF_SIZE 393216
#define UDP_MAX_PKT_SIZE 65536
#define UDP_HEADER_SIZE 8

typedef struct UDPContext {
    const AVClass *class;
    int flipflop;
    int pkt_count;
    int udp_fd;
    int udp_fd2;
    int ttl;
    int udplite_coverage;
    int buffer_size;
    int pkt_size;
    int is_multicast;
    int is_broadcast;
    int local_port;
    int local_port2;
    int reuse_socket;
    int overrun_nonfatal;
    struct sockaddr_storage dest_addr;
    struct sockaddr_storage dest_addr2;
    int dest_addr_len;
    int dest_addr_len2;
    int is_connected;

    /* Circular Buffer variables for use in UDP receive code */
    int circular_buffer_size;
    AVFifoBuffer *fifo;
    int circular_buffer_error;
    /* MP Receive queue*/
    int recv_buffer_size;
    AVFifoBuffer *recv_buffer;
    int recv_buffer_error;
    int64_t bitrate; /* number of bits to send per second */
    int64_t burst_bits;
    int close_req;
#if HAVE_PTHREAD_CANCEL
    pthread_t circular_buffer_thread;
    pthread_t recv_buffer_thread;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    pthread_mutex_t rb_mutex;
    pthread_cond_t rb_cond;
    int thread_started;
#endif
    uint8_t tmp[UDP_MAX_PKT_SIZE+4];
    uint8_t tmp2[UDP_MAX_PKT_SIZE+4];
    uint8_t raw_tmp1[UDP_MAX_PKT_SIZE+4];
    uint8_t raw_tmp2[UDP_MAX_PKT_SIZE+4];
    int remaining_in_dg;
    char *localaddr;
    char *localaddr2;
    int timeout;
    struct sockaddr_storage local_addr_storage;
    char *sources;
    char *block;
    IPSourceFilters filters;
    Graph graph;
    int dist[MAX_VERTEX_NUMBER + 1];
    int pre[MAX_VERTEX_NUMBER + 1];
} UDPContext;

#define OFFSET(x) offsetof(UDPContext, x)
#define D AV_OPT_FLAG_DECODING_PARAM
#define E AV_OPT_FLAG_ENCODING_PARAM

//The Consts that MPCM-QoE may use
#define BUFFER_SIZE_SET_SIZE 3
#define CHECKSUM_COV_SET_SIZE 3
#define PREDICT_SCOPE 5
static const int BufferSizeSet[] = {10000,100000,1000000};
static const int ChecksumCoverageSet[] = {20, 50, 1472};

static const AVOption options[] = {
    { "buffer_size",    "System data size (in bytes)",                     OFFSET(buffer_size),    AV_OPT_TYPE_INT,    { .i64 = -1 },    -1, INT_MAX, .flags = D|E },
    { "bitrate",        "Bits to send per second",                         OFFSET(bitrate),        AV_OPT_TYPE_INT64,  { .i64 = 0  },     0, INT64_MAX, .flags = E },
    { "burst_bits",     "Max length of bursts in bits (when using bitrate)", OFFSET(burst_bits),   AV_OPT_TYPE_INT64,  { .i64 = 0  },     0, INT64_MAX, .flags = E },
    { "localport",      "Local port",                                      OFFSET(local_port),     AV_OPT_TYPE_INT,    { .i64 = -1 },    -1, INT_MAX, D|E },
    { "local_port",     "Local port",                                      OFFSET(local_port),     AV_OPT_TYPE_INT,    { .i64 = -1 },    -1, INT_MAX, .flags = D|E },
    { "localaddr",      "Local address",                                   OFFSET(localaddr),      AV_OPT_TYPE_STRING, { .str = NULL },               .flags = D|E },
    { "udplite_coverage", "choose UDPLite head size which should be validated by checksum", OFFSET(udplite_coverage), AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, D|E },
    { "pkt_size",       "Maximum UDP packet size",                         OFFSET(pkt_size),       AV_OPT_TYPE_INT,    { .i64 = 1472 },  -1, INT_MAX, .flags = D|E },
    { "reuse",          "explicitly allow reusing UDP sockets",            OFFSET(reuse_socket),   AV_OPT_TYPE_BOOL,   { .i64 = -1 },    -1, 1,       D|E },
    { "reuse_socket",   "explicitly allow reusing UDP sockets",            OFFSET(reuse_socket),   AV_OPT_TYPE_BOOL,   { .i64 = -1 },    -1, 1,       .flags = D|E },
    { "broadcast", "explicitly allow or disallow broadcast destination",   OFFSET(is_broadcast),   AV_OPT_TYPE_BOOL,   { .i64 = 0  },     0, 1,       E },
    { "ttl",            "Time to live (multicast only)",                   OFFSET(ttl),            AV_OPT_TYPE_INT,    { .i64 = 16 },     0, INT_MAX, E },
    { "connect",        "set if connect() should be called on socket",     OFFSET(is_connected),   AV_OPT_TYPE_BOOL,   { .i64 =  0 },     0, 1,       .flags = D|E },
    { "fifo_size",      "set the UDP receiving circular buffer size, expressed as a number of packets with size of 188 bytes", OFFSET(circular_buffer_size), AV_OPT_TYPE_INT, {.i64 = 7*4096}, 0, INT_MAX, D },
    { "overrun_nonfatal", "survive in case of UDP receiving circular buffer overrun", OFFSET(overrun_nonfatal), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1,    D },
    { "timeout",        "set raise error timeout (only in read mode)",     OFFSET(timeout),        AV_OPT_TYPE_INT,    { .i64 = 0 },      0, INT_MAX, D },
    { "sources",        "Source list",                                     OFFSET(sources),        AV_OPT_TYPE_STRING, { .str = NULL },               .flags = D|E },
    { "block",          "Block list",                                      OFFSET(block),          AV_OPT_TYPE_STRING, { .str = NULL },               .flags = D|E },
    { NULL }
};

static const AVClass udp_class = {
    .class_name = "udp",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

static const AVClass udplite_context_class = {
    .class_name     = "udplite",
    .item_name      = av_default_item_name,
    .option         = options,
    .version        = LIBAVUTIL_VERSION_INT,
};

static int udp_set_multicast_ttl(int sockfd, int mcastTTL,
                                 struct sockaddr *addr)
{
#ifdef IP_MULTICAST_TTL
    if (addr->sa_family == AF_INET) {
        if (setsockopt(sockfd, IPPROTO_IP, IP_MULTICAST_TTL, &mcastTTL, sizeof(mcastTTL)) < 0) {
            ff_log_net_error(NULL, AV_LOG_ERROR, "setsockopt(IP_MULTICAST_TTL)");
            return -1;
        }
    }
#endif
#if defined(IPPROTO_IPV6) && defined(IPV6_MULTICAST_HOPS)
    if (addr->sa_family == AF_INET6) {
        if (setsockopt(sockfd, IPPROTO_IPV6, IPV6_MULTICAST_HOPS, &mcastTTL, sizeof(mcastTTL)) < 0) {
            ff_log_net_error(NULL, AV_LOG_ERROR, "setsockopt(IPV6_MULTICAST_HOPS)");
            return -1;
        }
    }
#endif
    return 0;
}

static int udp_join_multicast_group(int sockfd, struct sockaddr *addr,struct sockaddr *local_addr)
{
#ifdef IP_ADD_MEMBERSHIP
    if (addr->sa_family == AF_INET) {
        struct ip_mreq mreq;

        mreq.imr_multiaddr.s_addr = ((struct sockaddr_in *)addr)->sin_addr.s_addr;
        if (local_addr)
            mreq.imr_interface= ((struct sockaddr_in *)local_addr)->sin_addr;
        else
            mreq.imr_interface.s_addr= INADDR_ANY;
        if (setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const void *)&mreq, sizeof(mreq)) < 0) {
            ff_log_net_error(NULL, AV_LOG_ERROR, "setsockopt(IP_ADD_MEMBERSHIP)");
            return -1;
        }
    }
#endif
#if HAVE_STRUCT_IPV6_MREQ && defined(IPPROTO_IPV6)
    if (addr->sa_family == AF_INET6) {
        struct ipv6_mreq mreq6;

        memcpy(&mreq6.ipv6mr_multiaddr, &(((struct sockaddr_in6 *)addr)->sin6_addr), sizeof(struct in6_addr));
        //TODO: Interface index should be looked up from local_addr
        mreq6.ipv6mr_interface= 0;
        if (setsockopt(sockfd, IPPROTO_IPV6, IPV6_ADD_MEMBERSHIP, &mreq6, sizeof(mreq6)) < 0) {
            ff_log_net_error(NULL, AV_LOG_ERROR, "setsockopt(IPV6_ADD_MEMBERSHIP)");
            return -1;
        }
    }
#endif
    return 0;
}

static int udp_leave_multicast_group(int sockfd, struct sockaddr *addr,struct sockaddr *local_addr)
{
#ifdef IP_DROP_MEMBERSHIP
    if (addr->sa_family == AF_INET) {
        struct ip_mreq mreq;

        mreq.imr_multiaddr.s_addr = ((struct sockaddr_in *)addr)->sin_addr.s_addr;
        if (local_addr)
            mreq.imr_interface= ((struct sockaddr_in *)local_addr)->sin_addr;
        else
            mreq.imr_interface.s_addr= INADDR_ANY;
        if (setsockopt(sockfd, IPPROTO_IP, IP_DROP_MEMBERSHIP, (const void *)&mreq, sizeof(mreq)) < 0) {
            ff_log_net_error(NULL, AV_LOG_ERROR, "setsockopt(IP_DROP_MEMBERSHIP)");
            return -1;
        }
    }
#endif
#if HAVE_STRUCT_IPV6_MREQ && defined(IPPROTO_IPV6)
    if (addr->sa_family == AF_INET6) {
        struct ipv6_mreq mreq6;

        memcpy(&mreq6.ipv6mr_multiaddr, &(((struct sockaddr_in6 *)addr)->sin6_addr), sizeof(struct in6_addr));
        //TODO: Interface index should be looked up from local_addr
        mreq6.ipv6mr_interface= 0;
        if (setsockopt(sockfd, IPPROTO_IPV6, IPV6_DROP_MEMBERSHIP, &mreq6, sizeof(mreq6)) < 0) {
            ff_log_net_error(NULL, AV_LOG_ERROR, "setsockopt(IPV6_DROP_MEMBERSHIP)");
            return -1;
        }
    }
#endif
    return 0;
}

static int udp_set_multicast_sources(URLContext *h,
                                     int sockfd, struct sockaddr *addr,
                                     int addr_len, struct sockaddr_storage *local_addr,
                                     struct sockaddr_storage *sources,
                                     int nb_sources, int include)
{
    int i;
    if (addr->sa_family != AF_INET) {
#if HAVE_STRUCT_GROUP_SOURCE_REQ && defined(MCAST_BLOCK_SOURCE)
        /* For IPv4 prefer the old approach, as that alone works reliably on
         * Windows and it also supports supplying the interface based on its
         * address. */
        int i;
        for (i = 0; i < nb_sources; i++) {
            struct group_source_req mreqs;
            int level = addr->sa_family == AF_INET ? IPPROTO_IP : IPPROTO_IPV6;

            //TODO: Interface index should be looked up from local_addr
            mreqs.gsr_interface = 0;
            memcpy(&mreqs.gsr_group, addr, addr_len);
            memcpy(&mreqs.gsr_source, &sources[i], sizeof(*sources));

            if (setsockopt(sockfd, level,
                           include ? MCAST_JOIN_SOURCE_GROUP : MCAST_BLOCK_SOURCE,
                           (const void *)&mreqs, sizeof(mreqs)) < 0) {
                if (include)
                    ff_log_net_error(NULL, AV_LOG_ERROR, "setsockopt(MCAST_JOIN_SOURCE_GROUP)");
                else
                    ff_log_net_error(NULL, AV_LOG_ERROR, "setsockopt(MCAST_BLOCK_SOURCE)");
                return ff_neterrno();
            }
        }
        return 0;
#else
        av_log(h, AV_LOG_ERROR,
               "Setting multicast sources only supported for IPv4\n");
        return AVERROR(EINVAL);
#endif
    }
#if HAVE_STRUCT_IP_MREQ_SOURCE && defined(IP_BLOCK_SOURCE)
    for (i = 0; i < nb_sources; i++) {
        struct ip_mreq_source mreqs;
        if (sources[i].ss_family != AF_INET) {
            av_log(h, AV_LOG_ERROR, "Source/block address %d is of incorrect protocol family\n", i + 1);
            return AVERROR(EINVAL);
        }

        mreqs.imr_multiaddr.s_addr = ((struct sockaddr_in *)addr)->sin_addr.s_addr;
        if (local_addr)
            mreqs.imr_interface= ((struct sockaddr_in *)local_addr)->sin_addr;
        else
            mreqs.imr_interface.s_addr= INADDR_ANY;
        mreqs.imr_sourceaddr.s_addr = ((struct sockaddr_in *)&sources[i])->sin_addr.s_addr;

        if (setsockopt(sockfd, IPPROTO_IP,
                       include ? IP_ADD_SOURCE_MEMBERSHIP : IP_BLOCK_SOURCE,
                       (const void *)&mreqs, sizeof(mreqs)) < 0) {
            if (include)
                ff_log_net_error(h, AV_LOG_ERROR, "setsockopt(IP_ADD_SOURCE_MEMBERSHIP)");
            else
                ff_log_net_error(h, AV_LOG_ERROR, "setsockopt(IP_BLOCK_SOURCE)");
            return ff_neterrno();
        }
    }
#else
    return AVERROR(ENOSYS);
#endif
    return 0;
}
static int udp_set_url(URLContext *h,
                       struct sockaddr_storage *addr,
                       const char *hostname, int port)
{
    struct addrinfo *res0;
    int addr_len;

    res0 = ff_ip_resolve_host(h, hostname, port, SOCK_DGRAM, AF_UNSPEC, 0);
    if (!res0) return AVERROR(EIO);
    memcpy(addr, res0->ai_addr, res0->ai_addrlen);
    addr_len = res0->ai_addrlen;
    freeaddrinfo(res0);

    return addr_len;
}

static int udp_socket_create(URLContext *h, struct sockaddr_storage *addr,
                             socklen_t *addr_len, const char *localaddr)
{
    UDPContext *s = h->priv_data;
    int udp_fd = -1;
    struct addrinfo *res0, *res;
    int family = AF_UNSPEC;

    if (((struct sockaddr *) &s->dest_addr)->sa_family)
        family = ((struct sockaddr *) &s->dest_addr)->sa_family;
    res0 = ff_ip_resolve_host(h, (localaddr && localaddr[0]) ? localaddr : NULL,
                            s->local_port,
                            SOCK_DGRAM, family, AI_PASSIVE);
    if (!res0)
        goto fail;
    for (res = res0; res; res=res->ai_next) {
        if (s->udplite_coverage)
            udp_fd = ff_socket(res->ai_family, SOCK_DGRAM, IPPROTO_UDPLITE);
        else {
            udp_fd = ff_socket(res->ai_family, SOCK_DGRAM, 0);
        }
        if (udp_fd != -1) break;
        ff_log_net_error(NULL, AV_LOG_ERROR, "socket");
    }

    if (udp_fd < 0)
        goto fail;

    memcpy(addr, res->ai_addr, res->ai_addrlen);
    *addr_len = res->ai_addrlen;

    freeaddrinfo(res0);

    return udp_fd;

 fail:
    if (udp_fd >= 0)
        closesocket(udp_fd);
    if(res0)
        freeaddrinfo(res0);
    return -1;
}

static int udp_port(struct sockaddr_storage *addr, int addr_len)
{
    char sbuf[sizeof(int)*3+1];
    int error;

    if ((error = getnameinfo((struct sockaddr *)addr, addr_len, NULL, 0,  sbuf, sizeof(sbuf), NI_NUMERICSERV)) != 0) {
        av_log(NULL, AV_LOG_ERROR, "getnameinfo: %s\n", gai_strerror(error));
        return -1;
    }

    return strtol(sbuf, NULL, 10);
}


/**
 * If no filename is given to av_open_input_file because you want to
 * get the local port first, then you must call this function to set
 * the remote server address.
 *
 * url syntax: udp://host:port[?option=val...]
 * option: 'ttl=n'       : set the ttl value (for multicast only)
 *         'localport=n' : set the local port
 *         'pkt_size=n'  : set max packet size
 *         'reuse=1'     : enable reusing the socket
 *         'overrun_nonfatal=1': survive in case of circular buffer overrun
 *
 * @param h media file context
 * @param uri of the remote server
 * @return zero if no error.
 */
int ff_udp_set_remote_url(URLContext *h, const char *uri)
{
    UDPContext *s = h->priv_data;
    char hostname[256], buf[10];
    int port;
    const char *p;

    av_url_split(NULL, 0, NULL, 0, hostname, sizeof(hostname), &port, NULL, 0, uri);

    /* set the destination address */
    s->dest_addr_len = udp_set_url(h, &s->dest_addr, hostname, port);
    s->dest_addr_len = udp_set_url(h, &s->dest_addr2, hostname, port + 1000);
    if (s->dest_addr_len < 0) {
        return AVERROR(EIO);
    }
    s->is_multicast = ff_is_multicast_address((struct sockaddr*) &s->dest_addr);
    p = strchr(uri, '?');
    if (p) {
        if (av_find_info_tag(buf, sizeof(buf), "connect", p)) {
            int was_connected = s->is_connected;
            s->is_connected = strtol(buf, NULL, 10);
            if (s->is_connected && !was_connected) {
                if (connect(s->udp_fd, (struct sockaddr *) &s->dest_addr,
                            s->dest_addr_len)) {
                    s->is_connected = 0;
                    ff_log_net_error(h, AV_LOG_ERROR, "connect");
                    return AVERROR(EIO);
                }
            }
        }
    }

    return 0;
}

/**
 * Return the local port used by the UDP connection
 * @param h media file context
 * @return the local port number
 */
int ff_udp_get_local_port(URLContext *h)
{
    UDPContext *s = h->priv_data;
    return s->local_port;
}

/**
 * Return the udp file handle for select() usage to wait for several RTP
 * streams at the same time.
 * @param h media file context
 */
static int udp_get_file_handle(URLContext *h)
{
    UDPContext *s = h->priv_data;
    return s->udp_fd;
}

#if HAVE_PTHREAD_CANCEL

static void *udp_consumer(void *_URLContext){
    URLContext *h = _URLContext;
    UDPContext *s = h->priv_data;
    int old_cancelstate;

    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &old_cancelstate);
    pthread_mutex_lock(&s->rb_mutex);

    while(1) {
        int len;
        int len2;
        struct sockaddr_storage addr;
        socklen_t addr_len = sizeof(addr);
        struct sockaddr_storage addr2;
        socklen_t addr_len2 = sizeof(addr2);

        pthread_mutex_unlock(&s->rb_mutex);
        /* Blocking operations are always cancellation points;
           see "General Information" / "Thread Cancelation Overview"
           in Single Unix. */
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &old_cancelstate);
        len = recvfrom(s->udp_fd, s->raw_tmp1, sizeof(s->raw_tmp1), 0, (struct sockaddr *)&addr, &addr_len);
        len2 = recvfrom(s->udp_fd2, s->raw_tmp2, sizeof(s->raw_tmp2), 0, (struct sockaddr *)&addr2, &addr_len2);
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &old_cancelstate);
        pthread_mutex_lock(&s->rb_mutex);

        printf("===%d,%d===\n",len,len2);

        if (len < 0 || len2 < 0) {
            if (ff_neterrno() != AVERROR(EAGAIN) && ff_neterrno() != AVERROR(EINTR)) {
                s->recv_buffer_error = ff_neterrno();
                goto end;
            }
            continue;
        }
        if (ff_ip_check_source_lists(&addr, &s->filters))
            continue;

        /*
		if (s->flipflop == 1) {
			s->flipflop = 0;
		} else {
			s->flipflop = 1;
		}
		*/
        s->flipflop = 0;

        //++s->pkt_count;
        if (s->flipflop == 0) {
            if(av_fifo_space(s->recv_buffer) < len) { // not +4
            /* No Space left */
                if (s->overrun_nonfatal) {
                    av_log(h, AV_LOG_WARNING, "Receive buffer overrun. "
                            "Surviving due to overrun_nonfatal option\n");
                    continue;
                } else {
                    av_log(h, AV_LOG_ERROR, "Receive buffer overrun. "
                            "To avoid, increase fifo_size URL option. "
                            "To survive in such case, use overrun_nonfatal option\n");
                    s->recv_buffer_error = AVERROR(EIO);
                    goto end;
                }
            }
            av_fifo_generic_write(s->recv_buffer, s->raw_tmp1, len, NULL);
        } else {
            if(av_fifo_space(s->recv_buffer) < len2) {
            /* No Space left */
                if (s->overrun_nonfatal) {
                    av_log(h, AV_LOG_WARNING, "Receive buffer overrun. "
                            "Surviving due to overrun_nonfatal option\n");
                    continue;
                } else {
                    av_log(h, AV_LOG_ERROR, "Receive buffer overrun. "
                            "To avoid, increase fifo_size URL option. "
                            "To survive in such case, use overrun_nonfatal option\n");
                    s->recv_buffer_error = AVERROR(EIO);
                    goto end;
                }
            }
            av_fifo_generic_write(s->recv_buffer, s->raw_tmp2, len2, NULL);
        }
        pthread_cond_signal(&s->rb_cond);
    }

end:
    pthread_cond_signal(&s->rb_cond);
    pthread_mutex_unlock(&s->rb_mutex);
    return NULL;
}

static int isStartOfFrame(void* buf) {
	char *tbuf = (char*)buf;
	//printf("================%x,%x,%x,%x================",tbuf[0],tbuf[1],tbuf[2],tbuf[3]);
	if (tbuf[0] == 0 && tbuf[1] == 0 && tbuf[2] == 1){
		//printf("========================================\n====================================\n");
		return 1;
	}
	if (tbuf[0] == 0 && tbuf[1] == 0 && tbuf[2] == 0 && tbuf[3] == 1){
		//printf("========================================\n====================================\n");
		return 1;
	}

	return 0;
}

static void *circular_buffer_task_rx( void *_URLContext)
{
    URLContext *h = _URLContext;
    UDPContext *s = h->priv_data;
    int old_cancelstate;
    int nextMove;
    GraphNode nextNode;
    int bn,zn,un;
    int bn_tmp, bn_tmp_len;

    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &old_cancelstate);
    pthread_mutex_lock(&s->mutex);
    if (ff_socket_nonblock(s->udp_fd, 0) < 0) {
        av_log(h, AV_LOG_ERROR, "Failed to set blocking mode");
        s->circular_buffer_error = AVERROR(EIO);
        goto end;
    }
    while(1) {
        int read_step = 1000;
        struct sockaddr_storage addr;
        socklen_t addr_len = sizeof(addr);
        struct sockaddr_storage addr2;
        socklen_t addr_len2 = sizeof(addr2);

        pthread_mutex_unlock(&s->mutex);
        /* Blocking operations are always cancellation points;
           see "General Information" / "Thread Cancelation Overview"
           in Single Unix. */
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &old_cancelstate);
        //len = recvfrom(s->udp_fd, s->tmp+4, sizeof(s->tmp)-4, 0, (struct sockaddr *)&addr, &addr_len);
        //len2 = recvfrom(s->udp_fd2, s->tmp2+4, sizeof(s->tmp)-4, 0, (struct sockaddr *)&addr2, &addr_len2);
        while (av_fifo_size(s->recv_buffer) < read_step) {
        	//printf("%d\n",av_fifo_size(s->recv_buffer));
        }
        av_fifo_generic_read(s->recv_buffer, s->tmp+4, read_step, NULL);

        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &old_cancelstate);
        pthread_mutex_lock(&s->mutex);

        printf("=udp_task_rx==%d===\n",read_step);

        if (read_step < 0) {
            if (ff_neterrno() != AVERROR(EAGAIN) && ff_neterrno() != AVERROR(EINTR)) {
                s->circular_buffer_error = ff_neterrno();
                goto end;
            }
            continue;
        }
        if (ff_ip_check_source_lists(&addr, &s->filters))
            continue;
        //AV_WL32(s->tmp, len);
        //AV_WL32(s->tmp2, len2);
        AV_WL32(s->tmp, read_step);

        if(av_fifo_space(s->fifo) < read_step + 4) {
            /* No Space left */
            if (s->overrun_nonfatal) {
                av_log(h, AV_LOG_WARNING, "Circular buffer overrun. "
                        "Surviving due to overrun_nonfatal option\n");
                continue;
            } else {
                av_log(h, AV_LOG_ERROR, "Circular buffer overrun. "
                        "To avoid, increase fifo_size URL option. "
                        "To survive in such case, use overrun_nonfatal option\n");
                s->circular_buffer_error = AVERROR(EIO);
                goto end;
            }
        }

        //make run the algorithm at the edge of the Frame
        if (isStartOfFrame(s->tmp+4)) {
            //get the model prediction of next Scope

            //clear the edges
            graph_clear_edges(&(s->graph));
            //update the edge weghts

            //run the longest path algorithm
            nextMove = longestPath(s->graph, 0, s->dist, s->pre);
            nextNode = s->graph.vnodes[nextMove];
            //get the buffer size, chekcum coverage and protocol of next GOP
            bn = nextNode.b;
            zn = nextNode.z;
            un = nextNode.z == FULL_COV;
            //take the action and use Path B to cover the switch cost
            //switch the buffer size
            bn = 0;
            if (setsockopt(s->udp_fd, SOL_SOCKET, SO_RCVBUF, &bn, sizeof(bn)) < 0) {
				ff_log_net_error(h, AV_LOG_WARNING, "setsockopt(SO_RECVBUF)");
			}
			if (getsockopt(s->udp_fd, SOL_SOCKET, SO_RCVBUF, &bn_tmp, &bn_tmp_len) < 0) {
				ff_log_net_error(h, AV_LOG_WARNING, "getsockopt(SO_RCVBUF)");
			} else {
				av_log(h, AV_LOG_DEBUG, "end receive buffer size reported is %d\n", bn_tmp);
				if(bn_tmp < bn)
					av_log(h, AV_LOG_WARNING, "MPCM_QoE Algorithm attempted to set receive buffer to size %d but it only ended up set as %d\n", bn, bn_tmp);
			}
        }

        //only flip at the edge of Frame
        /*
        if (isStartOfFrame(s->tmp+4)) {
        	if (s->flipflop == 1) {
        		s->flipflop = 0;
        		printf("switch to path A. Frame code:%x,%x,%x,%x\n",s->tmp2[4],s->tmp2[5],s->tmp2[6],s->tmp2[7]);
        	} else if (s->flipflop == 0){
        		if (isStartOfFrame(s->tmp2+4)) {
        			s->flipflop = 1;
        			printf("switch to path B. Frame code:%x,%x,%x,%x\n",s->tmp[4],s->tmp[5],s->tmp[6],s->tmp[7]);
        		}
        	}
        }
        */

        //as soon as the other path start a new frame, switch to that path
        /*too much flucuation, too high frequency
        if (s->flipflop == 0) {
			if (isStartOfFrame(s->tmp2+4)) {
				s->flipflop = 1;
				printf("switch to path B. Frame code:%x,%x,%x,%x\n",s->tmp[4],s->tmp[5],s->tmp[6],s->tmp[7]);
			}
		} else if (s->flipflop == 1) {
			if (isStartOfFrame(s->tmp+4)) {
				s->flipflop = 0;
				printf("switch to path A. Frame code:%x,%x,%x,%x\n",s->tmp2[4],s->tmp2[5],s->tmp2[6],s->tmp2[7]);
			}
		}
		*/

        //as soon as the other path start a new key frame, switch to that path
        /*
		if (s->flipflop == 0) {
			if (isStartOfFrame(s->tmp2+4) && s->tmp2[8] == 65) {
				s->flipflop = 1;
				printf("switch to path B. Frame code:%x,%x,%x,%x\n",s->tmp2[4],s->tmp2[5],s->tmp2[6],s->tmp2[7]);
			}
		} else if (s->flipflop == 1) {
			if (isStartOfFrame(s->tmp+4) && s->tmp[8] == 65) {
				s->flipflop = 0;
				printf("switch to path A. Frame code:%x,%x,%x,%x\n",s->tmp[4],s->tmp[5],s->tmp[6],s->tmp[7]);
			}
		}


        //++s->pkt_count;
        if (s->flipflop == 0) {
        	av_fifo_generic_write(s->fifo, s->tmp, len+4, NULL);
        } else {
        	av_fifo_generic_write(s->fifo, s->tmp2, len2+4, NULL);
        }
        */
        av_fifo_generic_write(s->fifo, s->tmp, read_step+4, NULL);
        // tuning the RCVBUF
        /*
        int tmp = 0;
        int tmp1 = 20000;
        int tmp2 = 20000000;
        if (s->pkt_count*s->pkt_size % 10000000 == 0) {
        	if (tmp == tmp1) {
        		tmp = tmp1;
        	} else {
        		tmp = tmp2;
        	}
			if (setsockopt(s->udp_fd, SOL_SOCKET, SO_RCVBUF, &tmp, sizeof(tmp)) < 0) {
				ff_log_net_error(h, AV_LOG_WARNING, "setsockopt(SO_RECVBUF)");
			}
			len = sizeof(tmp);
			if (getsockopt(s->udp_fd, SOL_SOCKET, SO_RCVBUF, &tmp, &len) < 0) {
				ff_log_net_error(h, AV_LOG_WARNING, "getsockopt(SO_RCVBUF)");
			} else {
				av_log(h, AV_LOG_DEBUG, "end receive buffer size reported is %d\n", tmp);
				if(tmp < s->buffer_size)
					av_log(h, AV_LOG_WARNING, "attempted to set receive buffer to size %d but it only ended up set as %d\n", s->buffer_size, tmp);
			}
        }
        */
        pthread_cond_signal(&s->cond);
    }

end:
    pthread_cond_signal(&s->cond);
    pthread_mutex_unlock(&s->mutex);
    return NULL;
}

static void *circular_buffer_task_tx( void *_URLContext)
{
    URLContext *h = _URLContext;
    UDPContext *s = h->priv_data;
    int64_t target_timestamp = av_gettime_relative();
    int64_t start_timestamp = av_gettime_relative();
    int64_t sent_bits = 0;
    int64_t burst_interval = s->bitrate ? (s->burst_bits * 1000000 / s->bitrate) : 0;
    int64_t max_delay = s->bitrate ?  ((int64_t)h->max_packet_size * 8 * 1000000 / s->bitrate + 1) : 0;

    if (s->flipflop == 0) {
    	s->flipflop = 1;
    } else {
    	s->flipflop = 0;
    }

    pthread_mutex_lock(&s->mutex);

    if (ff_socket_nonblock(s->udp_fd, 0) < 0) {
        av_log(h, AV_LOG_ERROR, "Failed to set blocking mode");
        s->circular_buffer_error = AVERROR(EIO);
        goto end;
    }

    for(;;) {
        int len;
        const uint8_t *p;
        uint8_t tmp[4];
        int64_t timestamp;

        len=av_fifo_size(s->fifo);

        while (len<4) {
            if (s->close_req)
                goto end;
            if (pthread_cond_wait(&s->cond, &s->mutex) < 0) {
                goto end;
            }
            len=av_fifo_size(s->fifo);
        }

        av_fifo_generic_read(s->fifo, tmp, 4, NULL);
        len=AV_RL32(tmp);

        av_assert0(len >= 0);
        av_assert0(len <= sizeof(s->tmp));

        av_fifo_generic_read(s->fifo, s->tmp, len, NULL);

        pthread_mutex_unlock(&s->mutex);

        if (s->bitrate) {
            timestamp = av_gettime_relative();
            if (timestamp < target_timestamp) {
                int64_t delay = target_timestamp - timestamp;
                if (delay > max_delay) {
                    delay = max_delay;
                    start_timestamp = timestamp + delay;
                    sent_bits = 0;
                }
                av_usleep(delay);
            } else {
                if (timestamp - burst_interval > target_timestamp) {
                    start_timestamp = timestamp - burst_interval;
                    sent_bits = 0;
                }
            }
            sent_bits += len * 8;
            target_timestamp = start_timestamp + sent_bits * 1000000 / s->bitrate;
        }

        p = s->tmp;
        while (len) {
            int ret;
            av_assert0(len > 0);
            if (!s->is_connected) {
            	if (s->flipflop == 0) {
            		printf("path1=====================\n");
					ret = sendto (s->udp_fd, p, len, 0,
								(struct sockaddr *) &s->dest_addr,
								s->dest_addr_len);
            	} else {
            		printf("path2=====================\n");
            		ret = sendto (s->udp_fd2, p, len, 0,
            									(struct sockaddr *) &s->dest_addr2,
            									s->dest_addr_len2);
            	}

            } else
                ret = send(s->udp_fd, p, len, 0);
            if (ret >= 0) {
                len -= ret;
                p   += ret;
            } else {
                ret = ff_neterrno();
                if (ret != AVERROR(EAGAIN) && ret != AVERROR(EINTR)) {
                    pthread_mutex_lock(&s->mutex);
                    s->circular_buffer_error = ret;
                    pthread_mutex_unlock(&s->mutex);
                    return NULL;
                }
            }
        }

        pthread_mutex_lock(&s->mutex);
    }

end:
    pthread_mutex_unlock(&s->mutex);
    return NULL;
}


#endif

/* put it in UDP context */
/* return non zero if error */
static int udp_open(URLContext *h, const char *uri, int flags)
{
    char hostname[1024], localaddr[1024] = "", localaddr2[1024] = "";
    int port, udp_fd = -1, udp_fd2 = -1, tmp, bind_ret = -1, dscp = -1;
    UDPContext *s = h->priv_data;
    s->pkt_count = 0;
    int is_output;
    const char *p;
    char buf[256];
    struct sockaddr_storage my_addr;
    struct sockaddr_storage my_addr2;
    socklen_t len,len2;
    int bs[MAX_VERTEX_NUMBER+1], zs[MAX_VERTEX_NUMBER+1], ks[MAX_VERTEX_NUMBER+1];
    int gk,gb,gz;

    memset(bs, 0, sizeof(bs));
    memset(zs, 0, sizeof(zs));
    memset(ks, 0, sizeof(ks));

    //the step start from 1, step 0 is the auxilary source point
    for (gk; gk < PREDICT_SCOPE; ++gk){
        for (gb = 0; gb < BUFFER_SIZE_SET_SIZE; ++gb) {
            for (gz = 0; gz < CHECKSUM_COV_SET_SIZE; ++gz) {
                //a for the auxilary source vertex
                bs[1+gk * BUFFER_SIZE_SET_SIZE * CHECKSUM_COV_SET_SIZE + gb * CHECKSUM_COV_SET_SIZE + gz] = BufferSizeSet[gb];
                zs[1+gk * BUFFER_SIZE_SET_SIZE * CHECKSUM_COV_SET_SIZE + gb * CHECKSUM_COV_SET_SIZE + gz] = ChecksumCoverageSet[gz];
                ks[1+gk * BUFFER_SIZE_SET_SIZE * CHECKSUM_COV_SET_SIZE + gb * CHECKSUM_COV_SET_SIZE + gz] = gk + 1;               
            }
        }
    }

    
    //1 for the auxilary source vertex
    (s->graph) = construct_graph(1 + BUFFER_SIZE_SET_SIZE * CHECKSUM_COV_SET_SIZE * PREDICT_SCOPE,bs,zs,ks);
    for (gk = 0;gk < s->graph.V;++gk) {
        GraphNode node = s->graph.vnodes[gk];
        printf("%d,%d,%d\n",node.b, node.z, node.k);
    }

    h->is_streamed = 1;

    is_output = !(flags & AVIO_FLAG_READ);
    if (s->buffer_size < 0)
        s->buffer_size = is_output ? UDP_TX_BUF_SIZE : UDP_RX_BUF_SIZE;

    if (s->sources) {
        if (ff_ip_parse_sources(h, s->sources, &s->filters) < 0)
            goto fail;
    }

    if (s->block) {
        if (ff_ip_parse_blocks(h, s->block, &s->filters) < 0)
            goto fail;
    }

    if (s->pkt_size > 0)
        h->max_packet_size = s->pkt_size;

    p = strchr(uri, '?');
    if (p) {
        if (av_find_info_tag(buf, sizeof(buf), "reuse", p)) {
            char *endptr = NULL;
            s->reuse_socket = strtol(buf, &endptr, 10);
            /* assume if no digits were found it is a request to enable it */
            if (buf == endptr)
                s->reuse_socket = 1;
        }
        if (av_find_info_tag(buf, sizeof(buf), "overrun_nonfatal", p)) {
            char *endptr = NULL;
            s->overrun_nonfatal = strtol(buf, &endptr, 10);
            /* assume if no digits were found it is a request to enable it */
            if (buf == endptr)
                s->overrun_nonfatal = 1;
            if (!HAVE_PTHREAD_CANCEL)
                av_log(h, AV_LOG_WARNING,
                       "'overrun_nonfatal' option was set but it is not supported "
                       "on this build (pthread support is required)\n");
        }
        if (av_find_info_tag(buf, sizeof(buf), "ttl", p)) {
            s->ttl = strtol(buf, NULL, 10);
        }
        if (av_find_info_tag(buf, sizeof(buf), "udplite_coverage", p)) {
            s->udplite_coverage = strtol(buf, NULL, 10);
        }
        if (av_find_info_tag(buf, sizeof(buf), "localport", p)) {
            s->local_port = strtol(buf, NULL, 10);
            s->local_port2 = s->local_port + 1000;
        }
        if (av_find_info_tag(buf, sizeof(buf), "pkt_size", p)) {
            s->pkt_size = strtol(buf, NULL, 10);
        }
        if (av_find_info_tag(buf, sizeof(buf), "buffer_size", p)) {
            s->buffer_size = strtol(buf, NULL, 10);
        }
        if (av_find_info_tag(buf, sizeof(buf), "connect", p)) {
            s->is_connected = strtol(buf, NULL, 10);
        }
        if (av_find_info_tag(buf, sizeof(buf), "dscp", p)) {
            dscp = strtol(buf, NULL, 10);
        }
        if (av_find_info_tag(buf, sizeof(buf), "fifo_size", p)) {
            s->circular_buffer_size = strtol(buf, NULL, 10);
            if (!HAVE_PTHREAD_CANCEL)
                av_log(h, AV_LOG_WARNING,
                       "'circular_buffer_size' option was set but it is not supported "
                       "on this build (pthread support is required)\n");
        }
        if (av_find_info_tag(buf, sizeof(buf), "bitrate", p)) {
            s->bitrate = strtoll(buf, NULL, 10);
            if (!HAVE_PTHREAD_CANCEL)
                av_log(h, AV_LOG_WARNING,
                       "'bitrate' option was set but it is not supported "
                       "on this build (pthread support is required)\n");
        }
        if (av_find_info_tag(buf, sizeof(buf), "burst_bits", p)) {
            s->burst_bits = strtoll(buf, NULL, 10);
        }
        if (av_find_info_tag(buf, sizeof(buf), "localaddr", p)) {
            av_strlcpy(localaddr, buf, sizeof(localaddr));
        }
        if (av_find_info_tag(buf, sizeof(buf), "sources", p)) {
            if (ff_ip_parse_sources(h, buf, &s->filters) < 0)
                goto fail;
        }
        if (av_find_info_tag(buf, sizeof(buf), "block", p)) {
            if (ff_ip_parse_blocks(h, buf, &s->filters) < 0)
                goto fail;
        }
        if (!is_output && av_find_info_tag(buf, sizeof(buf), "timeout", p))
            s->timeout = strtol(buf, NULL, 10);
        if (is_output && av_find_info_tag(buf, sizeof(buf), "broadcast", p))
            s->is_broadcast = strtol(buf, NULL, 10);
    }
    /* handling needed to support options picking from both AVOption and URL */
    s->circular_buffer_size *= 188;
    /* set recv buffer size default 10MB*/
    s->recv_buffer_size = 10000000;


    if (flags & AVIO_FLAG_WRITE) {
        h->max_packet_size = s->pkt_size;
    } else {
        h->max_packet_size = UDP_MAX_PKT_SIZE;
    }
    h->rw_timeout = s->timeout;

    /* fill the dest addr */
    av_url_split(NULL, 0, NULL, 0, hostname, sizeof(hostname), &port, NULL, 0, uri);

    /* XXX: fix av_url_split */
    if (hostname[0] == '\0' || hostname[0] == '?') {
        /* only accepts null hostname if input */
        if (!(flags & AVIO_FLAG_READ))
            goto fail;
    } else {
        if (ff_udp_set_remote_url(h, uri) < 0)
            goto fail;
    }

    if ((s->is_multicast || s->local_port <= 0) && (h->flags & AVIO_FLAG_READ)) {
        s->local_port = port;
    	s->local_port2 = port +1000;
    }

    if (localaddr[0])
        udp_fd = udp_socket_create(h, &my_addr, &len, localaddr);
    else
        udp_fd = udp_socket_create(h, &my_addr, &len, s->localaddr);
    if (udp_fd < 0)
        goto fail;

    if (localaddr[0])
        udp_fd2 = udp_socket_create(h, &my_addr2, &len, localaddr);
    else
        udp_fd2 = udp_socket_create(h, &my_addr2, &len, s->localaddr);
    if (udp_fd2 < 0)
        goto fail;

    s->local_addr_storage=my_addr; //store for future multicast join

    /* Follow the requested reuse option, unless it's multicast in which
     * case enable reuse unless explicitly disabled.
     */
    if (s->reuse_socket > 0 || (s->is_multicast && s->reuse_socket < 0)) {
        s->reuse_socket = 1;
        if (setsockopt (udp_fd, SOL_SOCKET, SO_REUSEADDR, &(s->reuse_socket), sizeof(s->reuse_socket)) != 0)
            goto fail;
    }

    if (s->is_broadcast) {
#ifdef SO_BROADCAST
        if (setsockopt (udp_fd, SOL_SOCKET, SO_BROADCAST, &(s->is_broadcast), sizeof(s->is_broadcast)) != 0)
#endif
           goto fail;
    }

    /* Set the checksum coverage for UDP-Lite (RFC 3828) for sending and receiving.
     * The receiver coverage has to be less than or equal to the sender coverage.
     * Otherwise, the receiver will drop all packets.
     */
    if (s->udplite_coverage) {
        if (setsockopt (udp_fd, IPPROTO_UDPLITE, UDPLITE_SEND_CSCOV, &(s->udplite_coverage), sizeof(s->udplite_coverage)) != 0)
            av_log(h, AV_LOG_WARNING, "socket option UDPLITE_SEND_CSCOV not available");

        if (setsockopt (udp_fd, IPPROTO_UDPLITE, UDPLITE_RECV_CSCOV, &(s->udplite_coverage), sizeof(s->udplite_coverage)) != 0)
            av_log(h, AV_LOG_WARNING, "socket option UDPLITE_RECV_CSCOV not available");
    }

    if (dscp >= 0) {
        dscp <<= 2;
        if (setsockopt (udp_fd, IPPROTO_IP, IP_TOS, &dscp, sizeof(dscp)) != 0)
            goto fail;

    }

    /* If multicast, try binding the multicast address first, to avoid
     * receiving UDP packets from other sources aimed at the same UDP
     * port. This fails on windows. This makes sending to the same address
     * using sendto() fail, so only do it if we're opened in read-only mode. */
    if (s->is_multicast && (h->flags & AVIO_FLAG_READ)) {
        bind_ret = bind(udp_fd,(struct sockaddr *)&s->dest_addr, len);
    }
    /* bind to the local address if not multicast or if the multicast
     * bind failed */
    /* the bind is needed to give a port to the socket now */
    if (bind_ret < 0 && bind(udp_fd,(struct sockaddr *)&my_addr, len) < 0) {
        ff_log_net_error(h, AV_LOG_ERROR, "bind failed");
        goto fail;
    }
    udp_set_url(h, &my_addr2, hostname, s->local_port2);
    if (bind_ret < 0 && bind(udp_fd2,(struct sockaddr *)&my_addr2, len) < 0) {
		ff_log_net_error(h, AV_LOG_ERROR, "bind path 2 failed");
		goto fail;
	}

    len = sizeof(my_addr);
    len2 = sizeof(my_addr2);
    getsockname(udp_fd, (struct sockaddr *)&my_addr, &len);
    getsockname(udp_fd2, (struct sockaddr *)&my_addr2, &len2);
    s->local_port = udp_port(&my_addr, len);
    s->local_port2 = udp_port(&my_addr2, len2);

    if (s->is_multicast) {
        if (h->flags & AVIO_FLAG_WRITE) {
            /* output */
            if (udp_set_multicast_ttl(udp_fd, s->ttl, (struct sockaddr *)&s->dest_addr) < 0)
                goto fail;
        }
        if (h->flags & AVIO_FLAG_READ) {
            /* input */
            if (s->filters.nb_include_addrs) {
                if (udp_set_multicast_sources(h, udp_fd,
                                              (struct sockaddr *)&s->dest_addr,
                                              s->dest_addr_len, &s->local_addr_storage,
                                              s->filters.include_addrs,
                                              s->filters.nb_include_addrs, 1) < 0)
                    goto fail;
            } else {
                if (udp_join_multicast_group(udp_fd, (struct sockaddr *)&s->dest_addr,(struct sockaddr *)&s->local_addr_storage) < 0)
                    goto fail;
            }
            if (s->filters.nb_exclude_addrs) {
                if (udp_set_multicast_sources(h, udp_fd,
                                              (struct sockaddr *)&s->dest_addr,
                                              s->dest_addr_len, &s->local_addr_storage,
                                              s->filters.exclude_addrs,
                                              s->filters.nb_exclude_addrs, 0) < 0)
                    goto fail;
            }
        }
    }

    if (is_output) {
        /* limit the tx buf size to limit latency */
        tmp = s->buffer_size;
        if (setsockopt(udp_fd, SOL_SOCKET, SO_SNDBUF, &tmp, sizeof(tmp)) < 0) {
            ff_log_net_error(h, AV_LOG_ERROR, "setsockopt(SO_SNDBUF)");
            goto fail;
        }
    } else {
        /* set udp recv buffer size to the requested value (default 64K) */
        //tmp = s->buffer_size;
        tmp = 2000000;
        if (setsockopt(udp_fd, SOL_SOCKET, SO_RCVBUF, &tmp, sizeof(tmp)) < 0) {
            ff_log_net_error(h, AV_LOG_WARNING, "setsockopt(SO_RECVBUF)");
        }
        len = sizeof(tmp);
        if (getsockopt(udp_fd, SOL_SOCKET, SO_RCVBUF, &tmp, &len) < 0) {
            ff_log_net_error(h, AV_LOG_WARNING, "getsockopt(SO_RCVBUF)");
        } else {
            av_log(h, AV_LOG_DEBUG, "end receive buffer size reported is %d\n", tmp);
            if(tmp < s->buffer_size)
                av_log(h, AV_LOG_WARNING, "attempted to set receive buffer to size %d but it only ended up set as %d\n", s->buffer_size, tmp);
        }

        /* make the socket non-blocking */
        ff_socket_nonblock(udp_fd, 1);
    }
    if (s->is_connected) {
        if (connect(udp_fd, (struct sockaddr *) &s->dest_addr, s->dest_addr_len)) {
            ff_log_net_error(h, AV_LOG_ERROR, "connect");
            goto fail;
        }
    }

    s->udp_fd = udp_fd;
    s->udp_fd2 = udp_fd2;


#if HAVE_PTHREAD_CANCEL
    /*
      Create thread in case of:
      1. Input and circular_buffer_size is set
      2. Output and bitrate and circular_buffer_size is set
    */

    if (is_output && s->bitrate && !s->circular_buffer_size) {
        /* Warn user in case of 'circular_buffer_size' is not set */
        av_log(h, AV_LOG_WARNING,"'bitrate' option was set but 'circular_buffer_size' is not, but required\n");
    }

    if ((!is_output && s->circular_buffer_size) || (is_output && s->bitrate && s->circular_buffer_size)) {
        int ret;

        /* start the task going */
        s->fifo = av_fifo_alloc(s->circular_buffer_size);
        s->recv_buffer = av_fifo_alloc(s->recv_buffer_size);
		if (s->recv_buffer == NULL) {
			printf("\nFailed to alloc new recv buffer of size %d.\n",s->recv_buffer_size);
			exit(1);
		}

		int ret2;
		ret2 = pthread_mutex_init(&s->rb_mutex, NULL);
		if (ret2 != 0) {
		   av_log(h, AV_LOG_ERROR, "pthread_mutex_init failed : %s\n", strerror(ret2));
		   goto fail;
		}
		ret2 = pthread_cond_init(&s->rb_cond,NULL);
		if(ret2 != 0) {
			av_log(h,AV_LOG_ERROR, "pthread_cond_init rb failed %s\n", strerror(ret2));
			goto cond_fail;
		}
		ret2 = pthread_create(&s->recv_buffer_thread, NULL, udp_consumer, h);
		if(ret2 != 0) {
			av_log(h,AV_LOG_ERROR, "pthread_create rb failed %s\n", strerror(ret2));
			goto thread_fail;
		}
		s->thread_started = 1;

        ret = pthread_mutex_init(&s->mutex, NULL);
        if (ret != 0) {
            av_log(h, AV_LOG_ERROR, "pthread_mutex_init failed : %s\n", strerror(ret));
            goto fail;
        }
        ret = pthread_cond_init(&s->cond, NULL);
        if (ret != 0) {
            av_log(h, AV_LOG_ERROR, "pthread_cond_init failed : %s\n", strerror(ret));
            goto cond_fail;
        }
        ret = pthread_create(&s->circular_buffer_thread, NULL, is_output?circular_buffer_task_tx:circular_buffer_task_rx, h);
        if (ret != 0) {
            av_log(h, AV_LOG_ERROR, "pthread_create failed : %s\n", strerror(ret));
            goto thread_fail;
        }

    }
#endif

    return 0;
#if HAVE_PTHREAD_CANCEL
 thread_fail:
    pthread_cond_destroy(&s->cond);
    pthread_cond_destroy(&s->rb_cond);
 cond_fail:
    pthread_mutex_destroy(&s->mutex);
    pthread_mutex_destroy(&s->rb_mutex);
#endif
 fail:
    if (udp_fd >= 0)
        closesocket(udp_fd);
    av_fifo_freep(&s->fifo);
    av_fifo_freep(&s->recv_buffer);
    ff_ip_reset_filters(&s->filters);
    return AVERROR(EIO);
}

static int udplite_open(URLContext *h, const char *uri, int flags)
{
    UDPContext *s = h->priv_data;

    // set default checksum coverage
    s->udplite_coverage = UDP_HEADER_SIZE;

    return udp_open(h, uri, flags);
}

static int udp_read(URLContext *h, uint8_t *buf, int size)
{
    UDPContext *s = h->priv_data;
    int ret;
    struct sockaddr_storage addr;
    socklen_t addr_len = sizeof(addr);
#if HAVE_PTHREAD_CANCEL
    int avail, nonblock = h->flags & AVIO_FLAG_NONBLOCK;

    if (s->fifo) {
        pthread_mutex_lock(&s->mutex);
        do {
            avail = av_fifo_size(s->fifo);
            if (avail) { // >=size) {
                uint8_t tmp[4];

                av_fifo_generic_read(s->fifo, tmp, 4, NULL);
                avail= AV_RL32(tmp);
                if(avail > size){
                    av_log(h, AV_LOG_WARNING, "Part of datagram lost due to insufficient buffer size\n");
                    avail= size;
                }

                av_fifo_generic_read(s->fifo, buf, avail, NULL);
                av_fifo_drain(s->fifo, AV_RL32(tmp) - avail);
                pthread_mutex_unlock(&s->mutex);
                return avail;
            } else if(s->circular_buffer_error){
                int err = s->circular_buffer_error;
                pthread_mutex_unlock(&s->mutex);
                return err;
            } else if(nonblock) {
                pthread_mutex_unlock(&s->mutex);
                return AVERROR(EAGAIN);
            }
            else {
                /* FIXME: using the monotonic clock would be better,
                   but it does not exist on all supported platforms. */
                int64_t t = av_gettime() + 100000;
                struct timespec tv = { .tv_sec  =  t / 1000000,
                                       .tv_nsec = (t % 1000000) * 1000 };
                int err = pthread_cond_timedwait(&s->cond, &s->mutex, &tv);
                if (err) {
                    pthread_mutex_unlock(&s->mutex);
                    return AVERROR(err == ETIMEDOUT ? EAGAIN : err);
                }
                nonblock = 1;
            }
        } while( 1);
    }
#endif

    if (!(h->flags & AVIO_FLAG_NONBLOCK)) {
        ret = ff_network_wait_fd(s->udp_fd, 0);
        if (ret < 0)
            return ret;
    }
    printf("UDP_READ used.=================================\n");
    ret = recvfrom(s->udp_fd, buf, size, 0, (struct sockaddr *)&addr, &addr_len);
    if (ret < 0)
        return ff_neterrno();
    if (ff_ip_check_source_lists(&addr, &s->filters))
        return AVERROR(EINTR);
    return ret;
}

static int udp_write(URLContext *h, const uint8_t *buf, int size)
{
    UDPContext *s = h->priv_data;
    int ret;

    //printf("udp_write======================================\n");

#if HAVE_PTHREAD_CANCEL
    if (s->fifo) {
        uint8_t tmp[4];

        pthread_mutex_lock(&s->mutex);

        /*
          Return error if last tx failed.
          Here we can't know on which packet error was, but it needs to know that error exists.
        */
        if (s->circular_buffer_error<0) {
            int err=s->circular_buffer_error;
            pthread_mutex_unlock(&s->mutex);
            return err;
        }

        if(av_fifo_space(s->fifo) < size + 4) {
            /* What about a partial packet tx ? */
            pthread_mutex_unlock(&s->mutex);
            return AVERROR(ENOMEM);
        }
        AV_WL32(tmp, size);
        av_fifo_generic_write(s->fifo, tmp, 4, NULL); /* size of packet */
        av_fifo_generic_write(s->fifo, (uint8_t *)buf, size, NULL); /* the data */
        pthread_cond_signal(&s->cond);
        pthread_mutex_unlock(&s->mutex);
        return size;
    }
#endif
    if (!(h->flags & AVIO_FLAG_NONBLOCK)) {
        ret = ff_network_wait_fd(s->udp_fd, 1);
        if (ret < 0)
            return ret;
    }
    if (s->flipflop == 0) {
    	s->flipflop = 1;
    } else {
    	s->flipflop = 0;
    }
    if (!s->is_connected) {
   	        //int nal_type = buf[4] & 31;
    		printf("%x,%x,%x,%x,%x,%x\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
			ret = sendto (s->udp_fd, buf, size, 0,
									  (struct sockaddr *) &s->dest_addr,
									  s->dest_addr_len);

			//memcpy(s->dest_addr2, s->dest_addr, s->dest_addr_len);
			//(struct sockaddr_in *)s->dest_addr2.sin_port = htons(9000);
			udp_set_url(h,&s->dest_addr2,"127.0.0.1",9000);

			ret = sendto (s->udp_fd2, buf, size, 0,
									  (struct sockaddr *)&s->dest_addr2,
									  sizeof(s->dest_addr2));

    } else
        ret = send(s->udp_fd, buf, size, 0);

    return ret < 0 ? ff_neterrno() : ret;
}

static int udp_close(URLContext *h)
{
    UDPContext *s = h->priv_data;

#if HAVE_PTHREAD_CANCEL
    // Request close once writing is finished
    if (s->thread_started && !(h->flags & AVIO_FLAG_READ)) {
        pthread_mutex_lock(&s->mutex);
        s->close_req = 1;
        pthread_cond_signal(&s->cond);
        pthread_mutex_unlock(&s->mutex);
    }
#endif

    if (s->is_multicast && (h->flags & AVIO_FLAG_READ))
        udp_leave_multicast_group(s->udp_fd, (struct sockaddr *)&s->dest_addr,(struct sockaddr *)&s->local_addr_storage);
#if HAVE_PTHREAD_CANCEL
    if (s->thread_started) {
        int ret;
        // Cancel only read, as write has been signaled as success to the user
        if (h->flags & AVIO_FLAG_READ) {
#ifdef _WIN32
            /* recvfrom() is not a cancellation point for win32, so we shutdown
             * the socket and abort pending IO, subsequent recvfrom() calls
             * will fail with WSAESHUTDOWN causing the thread to exit. */
            shutdown(s->udp_fd, SD_RECEIVE);
            CancelIoEx((HANDLE)(SOCKET)s->udp_fd, NULL);
#else
            pthread_cancel(s->circular_buffer_thread);
#endif
        }
        ret = pthread_join(s->circular_buffer_thread, NULL);
        if (ret != 0)
            av_log(h, AV_LOG_ERROR, "pthread_join(): %s\n", strerror(ret));
        pthread_mutex_destroy(&s->mutex);
        pthread_cond_destroy(&s->cond);
    }
#endif
    closesocket(s->udp_fd);
    av_fifo_freep(&s->fifo);
    av_fifo_freep(&s->recv_buffer);
    ff_ip_reset_filters(&s->filters);
    return 0;
}

const URLProtocol ff_udp_protocol = {
    .name                = "udp",
    .url_open            = udp_open,
    .url_read            = udp_read,
    .url_write           = udp_write,
    .url_close           = udp_close,
    .url_get_file_handle = udp_get_file_handle,
    .priv_data_size      = sizeof(UDPContext),
    .priv_data_class     = &udp_class,
    .flags               = URL_PROTOCOL_FLAG_NETWORK,
};

const URLProtocol ff_udplite_protocol = {
    .name                = "udplite",
    .url_open            = udplite_open,
    .url_read            = udp_read,
    .url_write           = udp_write,
    .url_close           = udp_close,
    .url_get_file_handle = udp_get_file_handle,
    .priv_data_size      = sizeof(UDPContext),
    .priv_data_class     = &udplite_context_class,
    .flags               = URL_PROTOCOL_FLAG_NETWORK,
};

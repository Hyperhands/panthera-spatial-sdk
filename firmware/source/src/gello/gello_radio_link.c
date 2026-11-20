#include "gello_radio_link.h"
#include "core/controller.h"
#include <zephyr/net/socket.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#if defined(CONFIG_ROBOT_GELLO_RADIO_GATEWAY) && !defined(CONFIG_ROBOT_GELLO_RADIO_USB)
#include <zephyr/drivers/uart.h>
static const struct device *const uart=DEVICE_DT_GET(DT_ALIAS(gello_uart));
K_MSGQ_DEFINE(radio_rx,1,1024,1);
static atomic_t overflows;
#endif
#include <string.h>

#ifdef CONFIG_ROBOT_GELLO_WEB
#include "standalone/gello_web.h"
#ifdef CONFIG_ROBOT_GELLO_RADIO_USB
#include "gello_usb.h"
#endif
#ifdef CONFIG_ROBOT_GELLO_RADIO_GATEWAY
static struct k_spinlock web_lock;
static struct gw_packet web_forward;
static struct gw_board web_peer;
static uint64_t web_forward_at,web_peer_at;
#endif
#ifdef CONFIG_ROBOT_GELLO_RADIO_USB
static volatile uint32_t web_usb_requests,web_usb_failed,web_usb_deferred,web_usb_max_us;
static void web_usb_poll(uint64_t now,uint64_t last_good) {
    static uint64_t next;static uint32_t exchange;static unsigned mode=GW_RUN_MODE;
    if(now<next || !gello_usb_ready())return;
    /* Give the radio an unbroken recovery window after a missed/slow reply.
     * Beyond 200 ms input is already held, so diagnostics remain available. */
    if(last_good && now>=last_good && now-last_good>=10000 && now-last_good<200000) {
        web_usb_deferred++;return;
    }
    next=now+(mode==GW_RUN_MODE?1000000:100000);
    struct gw_packet q={.kind=GW_STATE,.mode=mode,.exchange=++exchange},reply;
    gello_web_snapshot(&q.board[1],now);
    k_spinlock_key_t key=k_spin_lock(&web_lock);
    q.board[0]=web_peer;uint64_t at=web_peer_at;
    k_spin_unlock(&web_lock,key);
    uint64_t age=at && now>=at?q.board[0].age_ms+(now-at)/1000:UINT32_MAX;
    q.board[0].age_ms=age>UINT32_MAX?UINT32_MAX:(uint32_t)age;
    uint64_t started=robot_us();web_usb_requests++;
    bool ok=gello_usb_web_exchange(&q,&reply);
    uint64_t elapsed=robot_us()-started;
    if(elapsed>web_usb_max_us)web_usb_max_us=(uint32_t)MIN(elapsed,UINT32_MAX);
    if(!ok)web_usb_failed++;
    else {mode=reply.mode;next=robot_us()+(mode==GW_RUN_MODE?1000000:100000);}
    if(ok && reply.op) {
        if(reply.node==12)gello_web_submit(&reply);
        else {
            key=k_spin_lock(&web_lock);web_forward=reply;web_forward_at=robot_us();k_spin_unlock(&web_lock,key);
        }
    }
}
#endif
#endif
#define LINK_PORT 5022
K_SEM_DEFINE(radio_begin,0,1);
static struct k_spinlock data_lock;
static struct gr_frame latest;
static uint64_t received_at;
static uint32_t requests,accepted,expired,rejected;
#ifdef CONFIG_ROBOT_GELLO_RADIO_USB
#include "gello_usb.h"
struct usb_request { struct gr_frame frame; uint64_t at; };
K_MSGQ_DEFINE(usb_queries,sizeof(struct usb_request),1,8);
K_MSGQ_DEFINE(usb_replies,sizeof(struct gr_frame),1,8);
/* Read-only diagnostics: distinguish scheduling/backlog from an exchange
 * failure. They do not change deadlines, queue sizes or sample validity. */
static volatile uint32_t usb_not_ready,usb_queue_expired,usb_exchange_failed;
static volatile uint32_t usb_late_reply,usb_reply_dropped,usb_query_dropped;
static volatile uint32_t usb_queue_max_us,usb_exchange_max_us;
static void usb_task(void *a,void *b,void *c) {
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    if(gello_usb_start()) { printk("GELLO radio USB init failed\n");return; }
#ifdef CONFIG_ROBOT_GELLO_WEB
    uint64_t last_good=0;
#endif
    for(;;) {
#ifdef CONFIG_ROBOT_GELLO_WEB
        web_usb_poll(robot_us(),last_good);
#endif
        struct usb_request q;struct gr_frame f;
        k_msgq_get(&usb_queries,&q,K_FOREVER);
        uint64_t now=robot_us();
        uint64_t queued=now>=q.at?now-q.at:UINT32_MAX;
        if(queued>usb_queue_max_us)usb_queue_max_us=(uint32_t)MIN(queued,UINT32_MAX);
        if(queued>=GR_MAX_RTT_US) { usb_queue_expired++;continue; }
        if(!gello_usb_ready()) { usb_not_ready++;continue; }
        uint64_t started=now;
        bool ok=gello_usb_exchange(&q.frame,&f);
        now=robot_us();
        uint64_t duration=now-started;
        if(duration>usb_exchange_max_us)usb_exchange_max_us=(uint32_t)MIN(duration,UINT32_MAX);
        if(!ok) { usb_exchange_failed++;continue; }
        if(!gr_matches(&f,q.frame.session,q.frame.sequence,q.at,now)) { usb_late_reply++;continue; }
        if(k_msgq_put(&usb_replies,&f,K_NO_WAIT))usb_reply_dropped++;
#ifdef CONFIG_ROBOT_GELLO_WEB
        last_good=now;
#endif
    }
}
K_THREAD_DEFINE(gello_usb_worker,4096,usb_task,NULL,NULL,NULL,7,0,0);
#endif
void gello_radio_start(void) { k_sem_give(&radio_begin); }
bool gello_radio_snapshot(struct gr_frame *out,uint64_t now) {
    k_spinlock_key_t key=k_spin_lock(&data_lock);
    *out=latest;uint64_t at=received_at;
    k_spin_unlock(&data_lock,key);
    return gr_age_snapshot(out,at,now,robot_us());
}
static void accept(struct gr_frame *f,uint64_t sent,uint64_t now) {
    gr_add_age(f,now-sent);
    k_spinlock_key_t key=k_spin_lock(&data_lock);latest=*f;received_at=now;k_spin_unlock(&data_lock,key);
    accepted++;
}
#if defined(CONFIG_ROBOT_GELLO_RADIO_GATEWAY) && !defined(CONFIG_ROBOT_GELLO_RADIO_USB)
static void uart_rx(const struct device *dev,void *unused) {
    ARG_UNUSED(unused);uart_irq_update(dev);
    int error=uart_err_check(dev);
    /* Drain at most one maximum frame plus overhead per invocation. */
    for(unsigned total=0;total<256 && uart_irq_rx_ready(dev);) {
        uint8_t bytes[32];int n=uart_fifo_read(dev,bytes,sizeof(bytes));if(n<=0)break;
        total+=(unsigned)n;
        for(int i=0;i<n;i++)if(error || k_msgq_put(&radio_rx,bytes+i,K_NO_WAIT))atomic_inc(&overflows);
    }
}
#endif
static void link_task(void *a,void *b,void *c) {
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);k_sem_take(&radio_begin,K_FOREVER);
    uint64_t session; k_spinlock_key_t key=k_spin_lock(&robot_lock);session=robot.boot;k_spin_unlock(&robot_lock,key);
    struct gr_parser parser={0};
    struct sockaddr_in peer={.sin_family=AF_INET,.sin_port=htons(LINK_PORT)};
    zsock_inet_pton(AF_INET,CONFIG_ROBOT_PEER_IPV4_ADDR,&peer.sin_addr);
    int fd=zsock_socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    struct sockaddr_in local={.sin_family=AF_INET,.sin_port=htons(LINK_PORT)};
    if(fd<0 || zsock_bind(fd,(struct sockaddr *)&local,sizeof(local))) {
        if(fd>=0)zsock_close(fd);
        printk("GELLO radio socket failed\n");return;
    }
#if defined(CONFIG_ROBOT_GELLO_RADIO_GATEWAY) && !defined(CONFIG_ROBOT_GELLO_RADIO_USB)
    if(!device_is_ready(uart) || uart_irq_callback_user_data_set(uart,uart_rx,NULL)) {
        printk("GELLO radio UART init failed\n");zsock_close(fd);return;
    }
    uart_irq_rx_enable(uart);
#else
    ARG_UNUSED(parser);
#endif
#if defined(CONFIG_ROBOT_GELLO_WEB) && defined(CONFIG_ROBOT_GELLO_RADIO_GATEWAY)
    uint64_t web_next=0,web_sent=0;uint32_t web_exchange=0;
#endif
    uint32_t seq=0;uint64_t sent=0,next=0,report=0;bool pending=false;
    for(;;) {
        uint64_t now=robot_us();
        struct gr_frame f;
#ifdef CONFIG_ROBOT_GELLO_RADIO_USB
        if(!k_msgq_get(&usb_replies,&f,K_NO_WAIT)) {
            now=robot_us();
            if(pending && gr_matches(&f,session,seq,sent,now)) { accept(&f,sent,now);pending=false; }
            else rejected++;
        }
#elif defined(CONFIG_ROBOT_GELLO_RADIO_GATEWAY)
        uint8_t byte;
        for(unsigned count=0;count<512 && !k_msgq_get(&radio_rx,&byte,K_NO_WAIT);count++) {
            if(gr_parse(&parser,byte,&f)) {
                now=robot_us();
                if(pending && gr_matches(&f,session,seq,sent,now)) { accept(&f,sent,now);pending=false; }
                else rejected++;
            }
        }
#endif
        for(unsigned i=0;i<4;i++) {
            uint8_t bytes[GR_SAMPLE_SIZE+1];struct sockaddr_in from;socklen_t size=sizeof(from);
            int n=zsock_recvfrom(fd,bytes,sizeof(bytes),ZSOCK_MSG_DONTWAIT,(struct sockaddr *)&from,&size);
            if(n<0)break;
            if(from.sin_addr.s_addr!=peer.sin_addr.s_addr || from.sin_port!=peer.sin_port) { rejected++;continue; }
#ifdef CONFIG_ROBOT_GELLO_WEB
            if(n==GW_SIZE && !memcmp(bytes,"GW04",4)) {
                struct gw_packet w;
                if(!gw_decode(bytes,n,&w))continue;
#ifdef CONFIG_ROBOT_GELLO_RADIO_GATEWAY
                if(w.kind==GW_PEER_REPLY && w.exchange==web_exchange && web_sent && robot_us()-web_sent<100000) {
                    key=k_spin_lock(&web_lock);web_peer=w.board[0];web_peer_at=web_sent;k_spin_unlock(&web_lock,key);
                }
#else
                if(w.kind==GW_PEER_QUERY) {
                    if(w.op)gello_web_submit(&w);
                    w.kind=GW_PEER_REPLY;w.op=0;gello_web_snapshot(&w.board[0],robot_us());
                    gw_encode(&w,bytes);(void)zsock_sendto(fd,bytes,GW_SIZE,ZSOCK_MSG_DONTWAIT,(struct sockaddr *)&from,size);
                }
#endif
                continue;
            }
#endif
            if(!gr_decode(bytes,(size_t)n,&f)) { rejected++;continue; }
            now=robot_us();
#ifdef CONFIG_ROBOT_GELLO_RADIO_GATEWAY
            if(f.kind!=GR_QUERY) { rejected++;continue; }
            uint64_t requester=f.session;uint32_t request=f.sequence;
            if(!gello_radio_snapshot(&f,now))continue;
            f.session=requester;f.sequence=request;
            size_t count=gr_encode(&f,bytes);
            (void)zsock_sendto(fd,bytes,count,ZSOCK_MSG_DONTWAIT,(struct sockaddr *)&from,size);
#else
            if(pending && gr_matches(&f,session,seq,sent,now)) { accept(&f,sent,now);pending=false; }
            else rejected++;
#endif
        }
        now=robot_us();
        if(pending && now-sent>=GR_MAX_RTT_US) { pending=false;expired++; }
        if(!pending && now>=next) {
            f=(struct gr_frame){.kind=GR_QUERY,.session=session,.sequence=++seq};
            uint8_t out[GR_SAMPLE_SIZE];size_t n=gr_encode(&f,out);
            sent=robot_us();pending=true;requests++;
#ifdef CONFIG_ROBOT_GELLO_RADIO_USB
            ARG_UNUSED(out);ARG_UNUSED(n);
            struct usb_request q={.frame=f,.at=sent};
            if(k_msgq_put(&usb_queries,&q,K_NO_WAIT)) { usb_query_dropped++;pending=false; }
#elif defined(CONFIG_ROBOT_GELLO_RADIO_GATEWAY)
            for(size_t i=0;i<n;i++)uart_poll_out(uart,out[i]);
#else
            if(zsock_sendto(fd,out,n,ZSOCK_MSG_DONTWAIT,(struct sockaddr *)&peer,sizeof(peer))!=(int)n)pending=false;
#endif
            next=sent+1000000/CONFIG_ROBOT_GELLO_RADIO_HZ;
        }
#if defined(CONFIG_ROBOT_GELLO_WEB) && defined(CONFIG_ROBOT_GELLO_RADIO_GATEWAY)
        if(now>=web_next) {
            struct gw_packet w;key=k_spin_lock(&web_lock);w=web_forward;uint64_t at=web_forward_at;k_spin_unlock(&web_lock,key);
            if(!at || now<at || now-at>500000)w.op=0;
            w.kind=GW_PEER_QUERY;w.exchange=++web_exchange;
            uint8_t bytes[GW_SIZE];gw_encode(&w,bytes);web_sent=now;web_next=now+100000;
            (void)zsock_sendto(fd,bytes,sizeof(bytes),ZSOCK_MSG_DONTWAIT,(struct sockaddr *)&peer,sizeof(peer));
        }
#endif
        if(now>=report) {
            /* Once per five seconds only, never in an ISR/control task. */
            printk("GELLO link requests=%u accepted=%u timeout=%u rejected=%u\n",requests,accepted,expired,rejected);
            report=now+5000000;
        }
        k_usleep(200);
    }
}
static void query_task(void *a,void *b,void *c) {
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    int fd=zsock_socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons(5021)};
    struct in_addr host;zsock_inet_pton(AF_INET,CONFIG_NET_CONFIG_PEER_IPV4_ADDR,&host);
    if(fd<0 || zsock_bind(fd,(struct sockaddr *)&addr,sizeof(addr))) { if(fd>=0)zsock_close(fd);return; }
    uint64_t last=0;
    for(;;) {
        uint8_t request[9],reply[GELLO_DUAL_REPLY_SIZE];struct sockaddr_in from;socklen_t size=sizeof(from);
        int n=zsock_recvfrom(fd,request,sizeof(request),0,(struct sockaddr *)&from,&size);
        uint64_t now=robot_us();
        if(n!=8 || from.sin_addr.s_addr!=host.s_addr || now-last<2000 ||
           (memcmp(request,"GQ01",4) && memcmp(request,"GQ02",4)))continue;
        last=now;struct gr_frame f;bool available=gello_radio_snapshot(&f,now);
        uint64_t boot; k_spinlock_key_t key=k_spin_lock(&robot_lock);boot=robot.boot;k_spin_unlock(&robot_lock,key);
        size_t count=gello_encode(reply,!memcmp(request,"GQ02",4),sys_get_le32(request+4),boot,
            f.scan,now,CONFIG_ROBOT_NODE_ID,available,f.samples,f.counters);
        (void)zsock_sendto(fd,reply,count,ZSOCK_MSG_DONTWAIT,(struct sockaddr *)&from,size);
    }
}
K_THREAD_DEFINE(gello_radio_thread,4096,link_task,NULL,NULL,NULL,7,0,0);
K_THREAD_DEFINE(gello_radio_query_thread,3072,query_task,NULL,NULL,NULL,8,0,0);

#include "gello/gello_web_codec.h"
#include "gello/gello_radio_codec.h"
#include "gello/gello_settings.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
int main(void) {
    struct gw_packet q={.kind=GW_COMMAND,.node=11,.op=GW_HAND,.session=123,.sequence=45,.expected_boot=456,
        .revision=7,.exchange=11,.action={.end={0,100,200,300,900,1000},.current_ma=1500,.close_at=100}},f;
    q.board[1]=(struct gw_board){.boot=456,.ack_session=123,.ack_sequence=44,.revision=7,.phase=4,.flags=255,
        .arm_mdeg={-200000,10000,300000,0,0,0},.hand_cdeg={-50,6000,0,0,0,0},.current_ma=1000,.age_ms=32};
    uint8_t wire[GW_SIZE];gw_encode(&q,wire);assert(gw_decode(wire,sizeof(wire),&f));
    assert(f.board[1].arm_mdeg[0]==-200000 && f.board[1].hand_cdeg[0]==-50 && f.action.end[5]==1000);
    for(unsigned i=0;i<GW_SIZE;i++)for(unsigned b=0;b<8;b++) {wire[i]^=1u<<b;assert(!gw_decode(wire,sizeof(wire),&f));wire[i]^=1u<<b;}
    for(unsigned i=0;i<GW_SIZE;i++)assert(!gw_decode(wire,i,&f));
    struct gw_parser p={0};for(unsigned i=0;i<333;i++)assert(!gw_parse(&p,0xab,&f));
    for(unsigned i=0;i<GW_SIZE;i++)assert(gw_parse(&p,wire[i],&f)==(i==GW_SIZE-1));
    q.action.end[0]=7501;gw_encode(&q,wire);assert(!gw_decode(wire,GW_SIZE,&f));
    q.action.end[0]=0;q.action.current_ma=2001;gw_encode(&q,wire);assert(!gw_decode(wire,GW_SIZE,&f));
    q.action.current_ma=500;q.expected_boot=0;gw_encode(&q,wire);assert(!gw_decode(wire,GW_SIZE,&f));
    /* A complete old GE01 response cannot be mistaken for a web command. */
    struct gr_frame old={.kind=GR_SAMPLE,.session=1,.source_boot=2};gr_encode(&old,wire);
    for(unsigned i=0;i<GW_SIZE;i++)assert(!gw_parse(&p,wire[i],&f));
    q=(struct gw_packet){.kind=GW_STATE,.mode=GW_SETTINGS_MODE};
    q.board[0].settings=3;q.board[0].home_phase=17;q.board[0].profile_crc=123456;
    gw_encode(&q,wire);assert(gw_decode(wire,sizeof(wire),&f));
    assert(f.mode==2 && f.board[0].settings==3 && f.board[0].home_phase==17 && f.board[0].profile_crc==123456);
    q.mode=4;gw_encode(&q,wire);assert(!gw_decode(wire,sizeof(wire),&f));
    for(unsigned flags=0;flags<16;flags++)for(unsigned freshness=0;freshness<4;freshness++) {
        struct gw_board boards[2]={{.settings=(flags&1)?1:0,.flags=(flags&2)?GW_PAUSED:0},
                                   {.settings=(flags&4)?1:0,.flags=(flags&8)?GW_PAUSED:0}};
        bool fresh[2]={freshness&1,freshness&2};
        assert(gw_mode_needs_reentry(GW_SETTINGS_MODE,boards,fresh)==
               ((fresh[0] && !(flags&1)) || (fresh[1] && !(flags&4))));
        assert(!gw_mode_needs_reentry(GW_EXIT_MODE,boards,fresh));
        assert(gw_mode_needs_reentry(GW_RUN_MODE,boards,fresh)==
               ((fresh[0] && (flags&3)) || (fresh[1] && (flags&12))));
        assert(gw_mode_open_ap(GW_ENTER_MODE,boards,fresh)==(flags==15 && freshness==3));
        assert(!gw_mode_open_ap(GW_RUN_MODE,boards,fresh));
        assert(gw_mode_finished(GW_EXIT_MODE,false,boards,fresh)==(!flags && freshness==3));
        assert(!gw_mode_finished(GW_EXIT_MODE,true,boards,fresh));
        for(unsigned side=0;side<2;side++) {
            assert(!gw_mode_action(GW_EXIT_MODE,true,boards,side));
            unsigned action=gw_mode_action(GW_EXIT_MODE,false,boards,side);
            if(action==GW_RESUME)assert(!(boards[0].settings|boards[1].settings));
        }
    }
    assert(gw_mode_press(GW_RUN_MODE)==GW_ENTER_MODE && gw_mode_press(GW_SETTINGS_MODE)==GW_EXIT_MODE);
    assert(gw_mode_press(GW_ENTER_MODE)==GW_EXIT_MODE && gw_mode_press(GW_EXIT_MODE)==GW_EXIT_MODE);
    /* A delayed ACK is not permission to issue an old movement again. */
    struct gw_packet request={.op=GW_HAND,.session=17,.sequence=23,.expected_boot=29};
    struct gw_board feedback={.boot=29};
    int64_t sent_at=10000000;
    for(int64_t age=0;age<GW_USER_ACK_US;age+=100000) {
        bool fresh=age<400000 || age>900000;
        assert(gw_delivery_check(&request,&feedback,sent_at+age,sent_at+GW_USER_ACK_US)==GW_DELIVERY_WAIT);
        assert(gw_delivery_can_send(sent_at+age,sent_at+GW_USER_SEND_US,fresh)==(fresh && age<GW_USER_SEND_US));
    }
    feedback.ack_session=17;feedback.ack_sequence=23;feedback.ack_status=GW_OK;
    assert(!gw_delivery_can_send(sent_at+1200000,sent_at+GW_USER_SEND_US,true));
    assert(gw_delivery_check(&request,&feedback,sent_at+1200000,sent_at+GW_USER_ACK_US)==GW_DELIVERY_ACK);
    feedback.boot++;
    assert(gw_delivery_check(&request,&feedback,sent_at+1200000,sent_at+GW_USER_ACK_US)==GW_DELIVERY_WAIT);
    feedback.boot--;feedback.ack_sequence++;
    assert(gw_delivery_check(&request,&feedback,sent_at+GW_USER_ACK_US,sent_at+GW_USER_ACK_US)==GW_DELIVERY_EXPIRED);
    feedback.ack_sequence--;feedback.ack_session++;
    assert(gw_delivery_check(&request,&feedback,sent_at+1200000,sent_at+GW_USER_ACK_US)==GW_DELIVERY_WAIT);
    feedback=(struct gw_board){.settings=GW_PROFILE_READY,.profile_crc=31};
    assert(gw_profile_needs_sync(GW_SETTINGS_MODE,true,&feedback,37));
    feedback.flags=GW_MANUAL;
    assert(!gw_profile_needs_sync(GW_SETTINGS_MODE,true,&feedback,37));
    feedback.flags|=GW_LEASE_EXPIRED;
    assert(!gw_profile_needs_sync(GW_SETTINGS_MODE,true,&feedback,37));
    feedback.flags=0;
    assert(!gw_profile_needs_sync(GW_RUN_MODE,true,&feedback,37));
    feedback.settings=0;
    assert(gw_profile_needs_sync(GW_RUN_MODE,true,&feedback,37));
    assert(!gw_profile_needs_sync(GW_RUN_MODE,false,&feedback,37));
    uint16_t legacy[6]={1000,1000,1000,1000,1000,1000};gw_migrate_legacy_profile(legacy);
    const uint16_t migrated[6]={1000,1000,611,678,667,633};assert(!memcmp(legacy,migrated,sizeof(legacy)));
    uint16_t partial[6]={1000,500,500,250,0,1000};gw_migrate_legacy_profile(partial);
    assert(partial[0]==1000 && partial[1]==500 && partial[2]==306 && partial[3]==169 && partial[4]==0 && partial[5]==633);
    /* Topology rides op 12 with the value in board[1].profile_crc; op 13
     * stays invalid and the relay-safe value field round-trips intact. */
    struct gw_packet topo={.kind=GW_COMMAND,.node=12,.op=GW_TOPOLOGY,.mode=GW_RUN_MODE,.session=9,.sequence=10,.expected_boot=77};
    topo.board[1].profile_crc=2;
    gw_encode(&topo,wire);assert(gw_decode(wire,sizeof(wire),&f));
    assert(f.op==GW_TOPOLOGY && f.board[1].profile_crc==2);
    struct gw_packet unknown={.kind=GW_COMMAND,.node=11,.op=13,.session=1,.sequence=1,.expected_boot=1};
    gw_encode(&unknown,wire);assert(!gw_decode(wire,sizeof(wire),&f));
    puts("Web codec: roundtrip, every bit/length, ranges and GE01 separation passed");
}

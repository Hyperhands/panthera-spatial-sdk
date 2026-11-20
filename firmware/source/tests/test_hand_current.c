#include "devices/hand_current.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int main(int argc,char **argv){
 assert(hand_current_fresh(true,true,1,2,100999,1000));
 assert(!hand_current_fresh(true,true,1,2,101000,1000));
 assert(!hand_current_fresh(true,false,1,2,1000,1000));
 assert(!hand_current_fresh(true,true,0,2,1000,1000));
 assert(!hand_current_fresh(false,true,1,2,1000,1000));
 assert(hand_current_fresh(true,true,1,3,1000,1000));
 assert(!hand_current_fresh(true,true,1,3,101000,1000));
 assert(!hand_current_fresh(true,true,1,1,1000,1000));
 assert(!hand_current_fresh(true,true,1,2,999,1000));
 hand_compliance_command_value_t v={0};configure_request_value_t base={0};
 v.duration_ms=20;v.control_feedforward_ma=500;base.hand_control_feedforward_ma=500;
 for(int i=0;i<6;i++){v.target_q_rad[i]=.1f*(i+1);v.joint_current_ma[i]=i<3?1000:100;base.hand_soft_current_ma[i]=i<3?500:100;}
 assert(hand_compliance_valid(&v,2));assert(!hand_compliance_valid(&v,3));
 hand_compliance_command_value_t teleop=v;
 teleop.control_feedforward_ma=2000;
 for(int i=0;i<6;i++)teleop.joint_current_ma[i]=2000;
 assert(hand_compliance_valid(&teleop,3));
 teleop.joint_current_ma[5]=1999;assert(!hand_compliance_valid(&teleop,3));teleop.joint_current_ma[5]=2000;
 teleop.control_feedforward_ma=2001;assert(!hand_compliance_valid(&teleop,3));teleop.control_feedforward_ma=2000;
 hand_command_value_t hand={0};configure_request_value_t cfg=base;uint8_t payload[28];
 hand.duration_ms=20;memcpy(hand.target_q_rad,v.target_q_rad,sizeof(hand.target_q_rad));
 cfg.hand_mode=3;cfg.hand_control_feedforward_ma=2000;
 assert(hand_control_payload(payload,&hand,&cfg)==14);
 assert(payload[0]==7&&payload[1]==208&&payload[2]==0&&payload[3]==100&&payload[12]==2&&payload[13]==88);
 /* The expiry path uses the original configuration: 2000 -> baseline500,
  * preserving the exact last commanded target angles. */
 uint8_t targets[12];memcpy(targets,payload+2,12);
 cfg.hand_control_feedforward_ma=500;cfg.hand_soft_current_ma[0]=2000;
 assert(hand_control_payload(payload,&hand,&cfg)==14);
 assert(payload[0]==1&&payload[1]==244&&!memcmp(payload+2,targets,12));
 cfg.hand_control_feedforward_ma=2001;assert(!hand_control_payload(payload,&hand,&cfg));
 cfg=base;cfg.hand_mode=2;
 assert(hand_control_payload(payload,&hand,&cfg)==28);
 assert(payload[0]==0&&payload[1]==20&&payload[14]==1&&payload[15]==244&&payload[26]==1&&payload[27]==244);
 cfg.hand_mode=1;cfg.hand_position_current_limit_ma[0]=300;
 assert(hand_control_payload(payload,&hand,&cfg)==26&&payload[14]==1&&payload[15]==44);
 hand.target_q_rad[0]=NAN;assert(!hand_control_payload(payload,&hand,&cfg));
 assert(hand_current_boosted(v.joint_current_ma,500,&base));
 assert(!hand_current_boosted(base.hand_soft_current_ma,500,&base));
 v.joint_current_ma[5]=2001;assert(!hand_compliance_valid(&v,2));v.joint_current_ma[5]=100;
 v.control_feedforward_ma=2001;assert(!hand_compliance_valid(&v,2));v.control_feedforward_ma=500;
 v.target_q_rad[5]=NAN;assert(!hand_compliance_valid(&v,2));v.target_q_rad[5]=.6f;
 v.has_header=v.has_control_lease=v.has_policy_revision=v.has_valid_for_us=true;
 v.has_target_q_rad=v.has_duration_ms=v.has_joint_current_ma=v.has_control_feedforward_ma=true;
 v.header.has_schema_revision=v.header.has_source_node=v.header.has_target_node=true;
 v.header.has_source_boot_id=v.header.has_target_boot_id=v.header.has_sequence=v.header.has_tx_monotonic_us=true;
 v.header.schema_revision=7;v.header.source_node=10;v.header.target_node=11;v.header.source_boot_id=123;v.header.target_boot_id=456;v.header.sequence=19;
 v.control_lease=789;v.policy_revision=2;
 uint8_t bytes[512];size_t n;assert(!hand_compliance_command_value_encode(&v,bytes,sizeof(bytes),&n));
 hand_compliance_command_value_t decoded;assert(!hand_compliance_command_value_decode(bytes,n,&decoded));
 assert(decoded.joint_current_ma[0]==1000&&decoded.joint_current_ma[5]==100&&decoded.duration_ms==20);
 assert(hand_compliance_command_value_decode(bytes,n-1,&decoded));
 if(argc>1){FILE*f=fopen(argv[1],"wb");assert(f);assert(fwrite(bytes,1,n,f)==n);fclose(f);}
 puts("PASS hand current validation and C wire roundtrip");return 0;
}

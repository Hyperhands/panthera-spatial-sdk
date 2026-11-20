#include <assert.h>
#include "../src/homing/local_home.c"
struct robot_state robot;
struct k_spinlock robot_lock;
int test_home_button;
static uint64_t now;
static bool inhibited;
uint64_t robot_us(void) { return now; }
bool gello_web_home_inhibited(void) { return inhibited; }
void robot_can_trace_enable(bool enabled) { (void)enabled; }
static void tick(void) { now+=4000;robot.hand_angle_at=now;local_home_tick(now); }
int main(void) {
    now=100000;local_home_init();
    robot.hand_feedback.online=true;robot.hand_feedback.valid_fields=2;
    inhibited=true;
    for(unsigned i=0;i<2000;i++){remote_button_pending=true;test_home_button=(i/20)%2;tick();}
    assert(local_home.phase==HOME_HAND_WAIT && !local_home.output && !remote_button_pending);
    assert(local_home.generation==0 && !robot.active);
    inhibited=false;test_home_button=0;
    for(unsigned i=0;i<2000 && local_home.phase==HOME_HAND_WAIT;i++)tick();
    assert(local_home.phase==HOME_WAIT && local_home.generation==1);
    /* Neither a physical nor paired request can restart an arm in settings. */
    inhibited=true;local_home.phase=HOME_STOPPED;uint32_t generation=local_home.generation;
    for(unsigned i=0;i<100;i++){remote_button_pending=true;test_home_button=(i/20)%2;tick();}
    assert(local_home.phase==HOME_STOPPED && local_home.generation==generation && !remote_button_pending);
    puts("Settings homing: boot hand wait, physical SW2 and paired requests inhibited; release permits normal startup");
}

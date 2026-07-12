#include "route_fsm.h"
#include "buzzer.h"
#include "control.h"
#include "motor.h"

#define STRAIGHT_M       1.0000f
#define DIAGONAL_M       1.280625f
#define STRAIGHT_SPEED   0.30f
#define DIAGONAL_SPEED   0.28f
#define LINE_SPEED       0.22f

typedef enum { SEG_STRAIGHT, SEG_DIAGONAL, SEG_LINE } SegmentType;
typedef struct { SegmentType type; } Segment;
static const Segment test1[]={{SEG_STRAIGHT}};
static const Segment test2[]={{SEG_STRAIGHT},{SEG_LINE},{SEG_STRAIGHT},{SEG_LINE}};
static const Segment test3[]={{SEG_DIAGONAL},{SEG_LINE},{SEG_DIAGONAL},{SEG_LINE}};
static RouteState state;
static const Segment *segments;
static unsigned segment_count,segment_index,lap,lap_target;
static bool point_event;

static void start_segment(void)
{
    switch(segments[segment_index].type){
    case SEG_STRAIGHT: Motion_DriveDistance(STRAIGHT_M,STRAIGHT_SPEED); break;
    case SEG_DIAGONAL: Motion_DriveDistance(DIAGONAL_M,DIAGONAL_SPEED); break;
    case SEG_LINE: Motion_FollowLine(LINE_SPEED); break;
    }
}

void Route_Init(void){state=ROUTE_SAFE;segments=0;segment_count=segment_index=lap=0;lap_target=1;point_event=false;}
bool Route_Start(RouteTest test)
{
    if(state==ROUTE_RUNNING)return false;
    switch(test){
    case ROUTE_TEST_1:segments=test1;segment_count=1;lap_target=1;break;
    case ROUTE_TEST_2:segments=test2;segment_count=4;lap_target=1;break;
    case ROUTE_TEST_3:segments=test3;segment_count=4;lap_target=1;break;
    case ROUTE_TEST_4:segments=test3;segment_count=4;lap_target=4;break;
    default:return false;
    }
    segment_index=0;lap=0;point_event=false;state=ROUTE_RUNNING;start_segment();return true;
}
void Route_Update(void)
{
    if(state!=ROUTE_RUNNING)return;
    if(Motion_HasFault()){Motor_Stop();state=ROUTE_ERROR;return;}
    if(!Motion_IsComplete())return;
    Buzzer_Beep(120);
    point_event=true;
    if(++segment_index<segment_count){start_segment();return;}
    segment_index=0;
    if(++lap>=lap_target){Motor_Stop();state=ROUTE_COMPLETE;return;}
    start_segment();
}
void Route_Abort(void){Motor_Stop();state=ROUTE_ERROR;}
RouteState Route_GetState(void){return state;}
unsigned Route_GetLap(void){return lap;}
bool Route_ConsumePointEvent(void){bool event=point_event;point_event=false;return event;}

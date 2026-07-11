#ifndef HCAR_ROUTE_FSM_H
#define HCAR_ROUTE_FSM_H
#include <stdbool.h>
typedef enum { ROUTE_TEST_1, ROUTE_TEST_2, ROUTE_TEST_3, ROUTE_TEST_4 } RouteTest;
typedef enum { ROUTE_SAFE, ROUTE_RUNNING, ROUTE_COMPLETE, ROUTE_ERROR } RouteState;
void Route_Init(void); bool Route_Start(RouteTest test); void Route_Update(void);
void Route_Abort(void); RouteState Route_GetState(void); unsigned Route_GetLap(void);
bool Route_ConsumePointEvent(void);
#endif

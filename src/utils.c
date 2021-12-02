/**
 * File for random helpers
 */

#ifndef _utils_c
#define _utils_c

#include "roboAI.h"

static int PRINT_SELF = 0;
static int PRINT_BALL = 1;
static int PRINT_OPP = 2;

static void print_blob_data(struct blob* some_blob) {
    printf("printing blob\n");
    assert(some_blob != NULL);
    printf("cx: %f, cy: %f\nvx: %f, vy: %f\nmx: %f, my:%f\ndx: %f, dy: %f\n", 
        some_blob->cx, some_blob->cy, some_blob->vx, some_blob->vy, 
        some_blob->mx, some_blob->my, some_blob->dx, some_blob->dy);
}

static void print_agent_data(struct RoboAI* ai, int agent_id) {
    if (agent_id == PRINT_SELF) {
        printf("printing SELF >>>>>>>\n");
        if (ai->st.self == NULL) {
            printf("not found\n");
        } else {
            printf("old_scx: %f, old_scy: %f\n svx: %f, svy: %f\n smx: %f, smy: %f\n sdx: %f, sdy: %f\n",
                ai->st.old_scx, ai->st.old_scy, ai->st.svx, ai->st.svy, ai->st.smx, ai->st.smy, ai->st.sdx, ai->st.sdy);
            print_blob_data(ai->st.self);
        }
    } else if (agent_id == PRINT_BALL) {
        printf("printing BALL >>>>>>>\n");
        if (ai->st.ball == NULL) {
            printf("not found\n");
        } else {
            printf("old_bcx: %f, old_bcy: %f\n bvx: %f, bvy: %f\n bmx: %f, bmy: %f\n bdx: %f, bdy: %f\n",
                ai->st.old_bcx, ai->st.old_bcy, ai->st.bvx, ai->st.bvy, ai->st.bmx, ai->st.bmy, ai->st.bdx, ai->st.bdy);
            print_blob_data(ai->st.ball);
        }
    } else if (agent_id == PRINT_OPP) {
        printf("printing OPP >>>>>>>\n");
        if (ai->st.opp == NULL) {
            printf("not found\n");
        } else {
            printf("old_ocx: %f, old_ocy: %f\n ovx: %f, ovy: %f\n omx: %f, omy: %f\n odx: %f, ody: %f\n",
                ai->st.old_ocx, ai->st.old_ocy, ai->st.ovx, ai->st.ovy, ai->st.omx, ai->st.omy, ai->st.odx, ai->st.ody);
            print_blob_data(ai->st.opp);
        }
    } else {
        assert(0); // line will always fail but its all good b.c we should never get here
    }

    printf("<<<<<<<<\n");
    fflush(stdout);
}

void print_self_data(struct RoboAI* ai) {
    print_agent_data(ai, PRINT_SELF);
}

void print_ball_data(struct RoboAI* ai) {
    print_agent_data(ai, PRINT_BALL);
}

void print_opp_data(struct RoboAI* ai) {
    print_agent_data(ai, PRINT_OPP);
}

#endif
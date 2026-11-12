#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "streamer.h"

/* Background worker for anything that can block: HTTP calls to the streamer
 * and shell commands. Jobs run one at a time, in order, on a second thread;
 * their completions are handed back to the main thread, which is the only one
 * that touches MAVLink state or sends messages.
 *
 *   work(job)  runs on the worker thread: do the slow part, store the outcome
 *   done(job)  runs on the main thread afterwards: send replies, update state
 */
struct job {
    void (*work)(struct job *);
    void (*done)(struct job *);
    struct job *next;

    /* who asked, for the reply */
    uint8_t req_sys, req_comp;
    int req_dst;
    uint16_t command;

    /* inputs and results; each job kind uses what it needs */
    int i;
    double d;
    float f;
    bool ok;
    char path[256];
    struct stream_props props;
    struct record_status rs;
    void *ptr;
};

bool worker_start(void);
/* readable whenever completed jobs are waiting for worker_dispatch() */
int worker_fd(void);
/* queue a copy of *j. false if the queue is full (the job is not run) */
bool worker_submit(const struct job *j);
/* main thread: run done() for every completed job */
void worker_dispatch(void);

#include "worker.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>

#define MAX_QUEUED 48

static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t wake = PTHREAD_COND_INITIALIZER;
static struct job *todo_head, *todo_tail;
static struct job *done_head, *done_tail;
static int queued;
static int pipe_fds[2] = { -1, -1 };

static void *run(void *arg)
{
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&lock);
        while (todo_head == NULL) {
            pthread_cond_wait(&wake, &lock);
        }
        struct job *j = todo_head;
        todo_head = j->next;
        if (todo_head == NULL) {
            todo_tail = NULL;
        }
        pthread_mutex_unlock(&lock);

        j->work(j);

        pthread_mutex_lock(&lock);
        j->next = NULL;
        if (done_tail != NULL) {
            done_tail->next = j;
        } else {
            done_head = j;
        }
        done_tail = j;
        pthread_mutex_unlock(&lock);

        const char byte = 1;
        ssize_t w;
        do {
            w = write(pipe_fds[1], &byte, 1);
        } while (w < 0 && errno == EINTR);   /* a full pipe is fine: the reader is already due */
    }
    return NULL;
}

bool worker_start(void)
{
    if (pipe(pipe_fds) != 0) {
        return false;
    }
    for (int i = 0; i < 2; i++) {
        fcntl(pipe_fds[i], F_SETFL, fcntl(pipe_fds[i], F_GETFL, 0) | O_NONBLOCK);
        fcntl(pipe_fds[i], F_SETFD, FD_CLOEXEC);
    }
    pthread_t t;
    if (pthread_create(&t, NULL, run, NULL) != 0) {
        return false;
    }
    pthread_detach(t);
    return true;
}

int worker_fd(void)
{
    return pipe_fds[0];
}

bool worker_submit(const struct job *j)
{
    struct job *copy = malloc(sizeof(*copy));
    if (copy == NULL) {
        return false;
    }
    *copy = *j;
    copy->next = NULL;

    pthread_mutex_lock(&lock);
    if (queued >= MAX_QUEUED) {
        pthread_mutex_unlock(&lock);
        free(copy);
        return false;
    }
    queued++;
    if (todo_tail != NULL) {
        todo_tail->next = copy;
    } else {
        todo_head = copy;
    }
    todo_tail = copy;
    pthread_cond_signal(&wake);
    pthread_mutex_unlock(&lock);
    return true;
}

void worker_dispatch(void)
{
    char buf[64];
    while (read(pipe_fds[0], buf, sizeof(buf)) > 0) {
    }
    for (;;) {
        pthread_mutex_lock(&lock);
        struct job *j = done_head;
        if (j != NULL) {
            done_head = j->next;
            if (done_head == NULL) {
                done_tail = NULL;
            }
            queued--;
        }
        pthread_mutex_unlock(&lock);
        if (j == NULL) {
            break;
        }
        if (j->done != NULL) {
            j->done(j);
        }
        free(j);
    }
}

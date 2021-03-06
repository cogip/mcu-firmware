/* System includes */
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

/* RIOT includes */
#include "mutex.h"
#include "xtimer.h"

/* Module includes */
#include "tracefd.h"
#include "tracefd_private.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

/**
 * @brief   Trace file descriptor parameters
 */
typedef struct {
    FILE *fd;                           /**< file descriptor */
    char filename[TRACEFD_MAX_PATH];    /**< filename */
    mutex_t lock;                       /**< lock protecting file access */
} tracefd_context_t;

/* Allocate memory for the tracefd contexts */
tracefd_context_t tracefd_contexts[TRACEFD_NUMOF_ALL];

/* Default tracefd descriptor for stderr */
tracefd_t tracefd_stderr = 0;

/* Default tracefd descriptor for stdout */
tracefd_t tracefd_stdout = 1;

/* Number of initialized tracefd */
tracefd_t tracefd_initialized = 0;

/* Files flusher thread PID */
kernel_pid_t files_flusher_pid = -1;

/* Files flusher thread stack */
static char files_flusher_thread_stack[THREAD_STACKSIZE_DEFAULT];

/* Flag set to true to make files flusher thread sleep */
bool sleep_files_flusher_thread = false;

static void *_thread_files_flusher(void *arg)
{
    (void)arg;
    for (;;) {
        if (sleep_files_flusher_thread) {
            sleep_files_flusher_thread = false;
            thread_sleep();
        }
        xtimer_ticks32_t loop_start_time = xtimer_now();
        tracefd_flush_all();
        xtimer_periodic_wakeup(&loop_start_time, TRACEFD_FLUSH_INTERVAL * US_PER_MS);
    }
    return EXIT_SUCCESS;
}

static inline void _check_initialized(void)
{
    if (tracefd_initialized == 0) {
        tracefd_init();
    }
}

void tracefd_init(void)
{
    assert(tracefd_initialized < TRACEFD_NUMOF_ALL);

    if (tracefd_initialized > 0) {
        return;
    }

    /* Init stderr descriptor */
    tracefd_context_t *stderr_context = &tracefd_contexts[tracefd_stderr];
    stderr_context->fd = stderr;
    mutex_init(&stderr_context->lock);
    tracefd_initialized++;

    /* Init stdout descriptor */
    tracefd_context_t *stdout_context = &tracefd_contexts[tracefd_stdout];
    stdout_context->fd = stdout;
    mutex_init(&stdout_context->lock);
    tracefd_initialized++;

    if (TRACEFD_NUMOF > 0 && tracefd_init_root_dir()) {
        files_flusher_pid = thread_create(
            files_flusher_thread_stack,
            sizeof(files_flusher_thread_stack),
            TRACEFD_THREAD_PRIORITY,
            THREAD_CREATE_SLEEPING,
            _thread_files_flusher,
            NULL,
            "Trace file flusher"
            );

        tracefd_start_files_flusher();
    }
}

tracefd_t tracefd_new(const char *filename)
{
    _check_initialized();
    assert(tracefd_initialized < TRACEFD_NUMOF_ALL);
    assert(strlen(TRACEFD_ROOT_DIR) + strlen(filename) + 1 < TRACEFD_MAX_PATH);

    tracefd_t id = tracefd_initialized;
    tracefd_context_t *context = &tracefd_contexts[id];
    sprintf(context->filename, "%s/%s", TRACEFD_ROOT_DIR, filename);
    context->fd = NULL;
    mutex_init(&context->lock);
    tracefd_initialized++;

    /* Create or truncate the file */
    FILE *fd = fopen(context->filename, "w");
    if (fd) {
        fclose(fd);
        return id;
    }
    else {
        return TRACEFD_NUMOF_ALL;
    }
}

void tracefd_open(const tracefd_t tracefd)
{
    assert(tracefd < tracefd_initialized);
    tracefd_context_t *context = &tracefd_contexts[tracefd];
    if (context->fd == NULL) {
        context->fd = fopen(context->filename, "a");
    }
}

void tracefd_close(const tracefd_t tracefd)
{
    assert(tracefd < tracefd_initialized);
    tracefd_context_t *context = &tracefd_contexts[tracefd];
    if (context->fd) {
        fclose(context->fd);
        context->fd = NULL;
    }
}

void tracefd_lock(const tracefd_t tracefd)
{
    _check_initialized();
    assert(tracefd < tracefd_initialized);
    tracefd_context_t *context = &tracefd_contexts[tracefd];
    mutex_lock(&context->lock);
}

void tracefd_unlock(const tracefd_t tracefd)
{
    _check_initialized();
    assert(tracefd < tracefd_initialized);
    tracefd_context_t *context = &tracefd_contexts[tracefd];
    mutex_unlock(&context->lock);
}

void tracefd_printf(const tracefd_t tracefd, const char *format, ...)
{
    _check_initialized();
    assert(tracefd < tracefd_initialized);
    const tracefd_context_t *context = &tracefd_contexts[tracefd];
    assert(context->fd);

    va_list argp;
    va_start(argp, format);
    vfprintf(context->fd, format, argp);
    va_end(argp);
    fflush(context->fd);
}

void tracefd_jlog(const tracefd_t tracefd, const char *format, ...)
{
    _check_initialized();
    assert(tracefd < tracefd_initialized);
    const tracefd_context_t *context = &tracefd_contexts[tracefd];
    assert(context->fd);

    tracefd_lock(tracefd);

    fprintf(context->fd, "{\"log\": \"");
    fflush(context->fd);

    va_list argp;
    va_start(argp, format);
    vfprintf(context->fd, format, argp);
    va_end(argp);
    fflush(context->fd);

    fprintf(context->fd, "\"}\n");
    fflush(context->fd);

    tracefd_unlock(tracefd);
}

void tracefd_flush(const tracefd_t tracefd)
{
    _check_initialized();
    assert(tracefd < tracefd_initialized);
    tracefd_context_t *context = &tracefd_contexts[tracefd];
    tracefd_lock(tracefd);
    if (context->fd) {
        tracefd_close(tracefd);
    }
    tracefd_open(tracefd);
    tracefd_unlock(tracefd);
}

void tracefd_flush_all(void)
{
    for (tracefd_t tracefd = TRACEFD_NUMOF_DEFAULT; tracefd < tracefd_initialized; tracefd++) {
        tracefd_flush(tracefd);
    }
}

void tracefd_start_files_flusher(void)
{
    thread_wakeup(files_flusher_pid);
}

void tracefd_stop_files_flusher(void)
{
    sleep_files_flusher_thread = true;
}

/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== pthread.c ========
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

/*
 *  For custom builds, this will always be defined, but it will
 *  not be defined when building just the library.  Make sure
 *  this goes ahead of _pthread.h.
 */
#ifndef ti_sysbios_posix_Settings_supportsMutexPriority__D
#define ti_sysbios_posix_Settings_supportsMutexPriority__D TRUE
#endif

#include "pthread.h"
#include "_pthread.h"
#include "errno.h"

void _pthread_runStub(UArg arg0, UArg arg1);

/*
 *  Default pthread attributes.  These are implementation
 *  dependent.
 */
static pthread_attr_t defaultPthreadAttrs = {
    1,       /* priority */
    NULL,    /* stack */
    0,       /* stacksize - would use Task_defaultStackSize but get compile
              *  error "expression must have a constant value"
              */
    0,       /*  guardsize */
    PTHREAD_CREATE_JOINABLE /* detachstate */
};

/*
 *************************************************************************
 *                      pthread_attr
 *************************************************************************
 */
/*
 *  ======== pthread_attr_destroy ========
 */
int pthread_attr_destroy(pthread_attr_t *attr)
{
    return (0);
}

/*
 *  ======== pthread_attr_getdetachstate ========
 */
int pthread_attr_getdetachstate(const pthread_attr_t *attr, int *detachstate)
{
    *detachstate = attr->detachstate;
    return (0);
}

/*
 *  ======== pthread_attr_getguardsize ========
 */
int pthread_attr_getguardsize(const pthread_attr_t *attr, size_t *guardsize)
{
    *guardsize = attr->guardsize;
    return (0);
}

/*
 *  ======== pthread_attr_getschedparam ========
 */
int pthread_attr_getschedparam(const pthread_attr_t *attr,
        struct sched_param *schedparam)
{
    schedparam->sched_priority = attr->priority;
    return (0);
}

/*
 *  ======== pthread_attr_getstack ========
 */
int pthread_attr_getstack (const pthread_attr_t *attr, void **stackaddr,
        size_t *stacksize)
{
    *stackaddr = attr->stack;
    *stacksize = attr->stacksize;
    return (0);
}

/*
 *  ======== pthread_attr_getstacksize ========
 */
int pthread_attr_getstacksize(const pthread_attr_t *attr, size_t *stacksize)
{
    *stacksize = attr->stacksize;
    return (0);
}

/*
 *  ======== pthread_attr_init ========
 */
int pthread_attr_init(pthread_attr_t *attr)
{
    *attr = defaultPthreadAttrs;
    attr->stacksize = Task_defaultStackSize;

    return (0);
}

/*
 *  ======== pthread_attr_setdetachstate ========
 */
int pthread_attr_setdetachstate(pthread_attr_t *attr, int detachstate)
{
    if ((detachstate != PTHREAD_CREATE_JOINABLE) &&
            (detachstate != PTHREAD_CREATE_DETACHED)) {
        return (EINVAL);
    }

    attr->detachstate = detachstate;
    return (0);
}

/*
 *  ======== pthread_attr_setguardsize ========
 */
int pthread_attr_setguardsize(pthread_attr_t *attr, size_t guardsize)
{
    attr->guardsize = guardsize;
    return (0);
}

/*
 *  ======== pthread_attr_setschedparam ========
 */
int pthread_attr_setschedparam(pthread_attr_t *attr,
        const struct sched_param *schedparam)
{
    int     priority = schedparam->sched_priority;

    if ((priority >= (int)Task_numPriorities) || (priority == 0) ||
            (priority < -1)) {
        /* Bad priority value */
        return (EINVAL);
    }
    attr->priority = priority;
    return (0);
}

/*
 *  ======== pthread_attr_setstack ========
 */
int pthread_attr_setstack (pthread_attr_t *attr, void *stackaddr,
        size_t stacksize)
{
    /*
     *  Don't return an error if stack is not aligned. Task_create()
     *  will adjust the stack address to be aligned.
     */
    attr->stack = stackaddr;
    attr->stacksize = stacksize;
    return (0);
}

/*
 *  ======== pthread_attr_setstacksize ========
 */
int pthread_attr_setstacksize(pthread_attr_t *attr, size_t stacksize)
{
    attr->stacksize = stacksize;
    return (0);
}

/*
 *************************************************************************
 *                      pthread
 *************************************************************************
 */

/*
 *  ======== pthread_cancel ========
 *  The specification of this API is that it be used as a means for one thread
 *  to termintate the execution of another thread.  There is no mention of
 *  returning an error if the argument, pthread, is the same thread as the
 *  calling thread.
 */
int pthread_cancel(pthread_t pthread)
{
    pthread_Obj  *thread = (pthread_Obj *)pthread;
    UInt          key;

    /*
     *  Cancel the thread.  Only asynchronous cancellation is supported,
     *  since functions that would normally be cancellation points (eg,
     *  printf()), are not cancellation points for BIOS.
     */
    key = Task_disable();

    /* Indicate that cancellation is requested. */
    thread->cancelPending = 1;

    if (thread->cancelState == PTHREAD_CANCEL_ENABLE) {
        /* Set this task's priority to -1 to stop it from running. */
        Task_setPri(thread->task, -1);

        Task_restore(key);

        /* Pop and execute the cleanup handlers */
        while (thread->cleanupList != NULL) {
            _pthread_cleanup_pop(thread->cleanupList, 1);
        }

        /* Cleanup any pthread specific data */
        _pthread_removeThreadKeys(pthread);

        if (thread->detached) {
            /* Free memory */
#if ti_sysbios_posix_Settings_supportsMutexPriority__D
            Queue_destruct(&(thread->mutexList));
#endif
            Semaphore_destruct(&(thread->joinSem));
            Task_delete(&(thread->task));

            Memory_free(Task_Object_heap(), thread, sizeof(pthread_Obj));
        }
        else {
            /* pthread_join() will clean up. */
            thread->ret = PTHREAD_CANCELED;
            Semaphore_post(Semaphore_handle(&(thread->joinSem)));
        }
    }
    else {
        Task_restore(key);
    }

    return (0);
}

/*
 *  ======== pthread_create ========
 */
int pthread_create(pthread_t *newthread, const pthread_attr_t *attr,
        void *(*startroutine)(void *), void *arg)
{
    Semaphore_Params  semParams;
    Task_Params       taskParams;
    pthread_Obj      *thread = NULL;
    Error_Block       eb;
    pthread_attr_t   *pAttr;

    Error_init(&eb);
    Task_Params_init(&taskParams);

    *newthread = NULL;

    thread = (pthread_Obj *)Memory_alloc(Task_Object_heap(),
            sizeof(pthread_Obj), 0, &eb);

    if (thread == NULL) {
        return (ENOMEM);
    }

    pAttr = (attr == NULL) ? &defaultPthreadAttrs : (pthread_attr_t *)attr;

    taskParams.priority = pAttr->priority;
    taskParams.stack = pAttr->stack;
    taskParams.stackSize = pAttr->stacksize + pAttr->guardsize;

    /* Save the function in arg0 for ROV */
    taskParams.arg0 = (UArg)startroutine;
    taskParams.arg1 = (UArg)thread;
    taskParams.env = arg;
    taskParams.priority = -1;

    thread->detached = (pAttr->detachstate == PTHREAD_CREATE_JOINABLE) ? 0 : 1;
    thread->fxn = startroutine;
    thread->joinThread = NULL;
    thread->cancelState = PTHREAD_CANCEL_ENABLE;
    thread->cancelPending = 0;
    thread->priority = pAttr->priority;
    thread->cleanupList = NULL;

#if ti_sysbios_posix_Settings_supportsMutexPriority__D
    thread->blockedMutex = NULL;
    Queue_elemClear((Queue_Elem *)thread);
    Queue_construct(&(thread->mutexList), NULL);
#endif

    /* List of keys for which thread has called pthread_setspecific() */
    Queue_construct(&(thread->keyList), NULL);

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;

    Semaphore_construct(&(thread->joinSem), 0, &semParams);

    thread->task = Task_create((Task_FuncPtr)_pthread_runStub,
            &taskParams, &eb);

    if (thread->task == NULL) {
        Semaphore_destruct(&(thread->joinSem));

#if ti_sysbios_posix_Settings_supportsMutexPriority__D
        Queue_destruct(&(thread->mutexList));
#endif
        Memory_free(Task_Object_heap(), thread, sizeof(pthread_Obj));

        return (ENOMEM);
    }

    *newthread = (pthread_t)thread;
    Task_setPri(thread->task, pAttr->priority);

    return (0);
}

/*
 *  ======== pthread_detach ========
 */
int pthread_detach(pthread_t pthread)
{
    pthread_Obj  *thread = (pthread_Obj *)pthread;
    UInt          key;

    key = Task_disable();

    if ((thread->joinThread != NULL) || (thread->detached)) {
        Task_restore(key);

        /*
         *  A thread has already called pthread_join() or
         *  the thread is already detached.
         */
        return (EINVAL);
    }

    /*
     *  pthread_detach() marks the thread as detached.  When
     *  the thread terminates, its resources will automatically
     *  be freed without the need for another thread to call
     *  pthread_join().
     */
    thread->detached = 1;

    Task_restore(key);

    return (0);
}

/*
 *  ======== pthread_equal ========
 */
int pthread_equal(pthread_t pt1, pthread_t pt2)
{
    return (pt1 == pt2);
}

/*
 *  ======== pthread_exit ========
 */
void pthread_exit(void *retval)
{
    pthread_Obj      *thread = (pthread_Obj *)pthread_self();

    /*
     *  This function terminates the calling thread and returns
     *  a value via retval that (if the thread is joinable) is available to
     *  another thread that calls pthread_join().
     *
     *  Any clean-up handlers that have not yet been popped, are popped
     *  (in the reverse of the order in which they were pushed) and executed.
     */
    thread->ret = retval;

    /*
     *  Don't bother disabling the Task scheduler while the thread
     *  is exiting.  It will be up to the application to not make
     *  such calls as pthread_cancel() or pthread_detach() while the
     *  thread is exiting.
     */

    /* Pop and execute the cleanup handlers */
    while (thread->cleanupList != NULL) {
        _pthread_cleanup_pop(thread->cleanupList, 1);
    }

    /* Cleanup any pthread specific data */
    _pthread_removeThreadKeys((pthread_t)thread);

    if (!thread->detached) {
        Semaphore_post(Semaphore_handle(&(thread->joinSem)));

        /* Set this task's priority to -1 to stop it from running. */
        Task_setPri(thread->task, -1);
    }
    else {
        /* Free memory */
#if ti_sysbios_posix_Settings_supportsMutexPriority__D
        Queue_destruct(&(thread->mutexList));
#endif
        Semaphore_destruct(&(thread->joinSem));

        Memory_free(Task_Object_heap(), thread, sizeof(pthread_Obj));

        /*
         *  Don't call Task_delete on the calling thread.  Task_exit()
         *  will put the task on the terminated queue, and if
         *  Task_deleteTerminatedTasks is TRUE, the task will be cleaned
         *  up automatically.
         */
        Task_exit();
    }
}

/*
 *  ======== pthread_getschedparam ========
 */
int pthread_getschedparam(pthread_t pthread, int *policy,
        struct sched_param *param)
{
    pthread_Obj      *thread = (pthread_Obj *)pthread;

    *policy = SCHED_OTHER;

    /*
     *  Note: This may not be the priority that the task is running
     *  at (eg, if it is holding a mutex with a higher priority
     *  ceiling).
     */
    param->sched_priority = thread->priority;

    return (0);
}

/*
 *  ======== pthread_join ========
 *  Wait for thread to terminate.
 *
 *  If multiple threads simultaneously try to join with the same
 *  thread, the results are undefined.  We will return an error.
 *
 *  If the thread calling pthread_join() is canceled, then the target
 *  thread will remain joinable (i.e., it will not be detached).
 */
int pthread_join(pthread_t pthread, void **thread_return)
{
    pthread_Obj  *thread = (pthread_Obj *)pthread;
    UInt          key;

    key = Task_disable();

    if ((thread->joinThread != NULL) || (thread->detached != 0)) {
        /*
         *  Error - Another thread has already called pthread_join()
         *  for this thread, or the thread is in the detached state.
         */
        Task_restore(key);
        return (EINVAL);
    }

    if (pthread == pthread_self()) {
        Task_restore(key);
        return (EDEADLK);
    }

    /*
     *  Allow pthread_join() to be called from a BIOS Task.  If we
     *  set joinThread to pthread_self(), we could get NULL if the
     *  Task arg1 is 0.  All we need is a non-NULL value for joinThread.
     */
    thread->joinThread = Task_self();

    Task_restore(key);

    Semaphore_pend(Semaphore_handle(&(thread->joinSem)), BIOS_WAIT_FOREVER);

    if (thread_return) {
        *thread_return = thread->ret;
    }

#if ti_sysbios_posix_Settings_supportsMutexPriority__D
    Queue_destruct(&(thread->mutexList));
#endif
    Semaphore_destruct(&(thread->joinSem));

    Task_delete(&(thread->task));

    Memory_free(Task_Object_heap(), thread, sizeof(pthread_Obj));
    return (0);
}

/*
 *  ======== pthread_once ========
 */
int pthread_once(pthread_once_t *once, void (*initFxn)(void))
{
    UInt    key;

    key = Task_disable();

    if (*once == PTHREAD_ONCE_INIT) {
        (*initFxn)();
        *once = ~PTHREAD_ONCE_INIT;
    }

    Task_restore(key);

    return (0);
}

/*
 *  ======== pthread_self ========
 */
pthread_t pthread_self(void)
{
    return ((pthread_t)(xdc_uargToPtr(Task_getArg1(Task_self()))));
}

/*
 *  ======== pthread_setcancelstate ========
 */
int pthread_setcancelstate(int state, int *oldstate)
{
    pthread_Obj *thread = (pthread_Obj *)pthread_self();

    if ((state != PTHREAD_CANCEL_ENABLE) &&
            (state != PTHREAD_CANCEL_DISABLE)) {
        return (EINVAL);
    }

    *oldstate = thread->cancelState;

    thread->cancelState = state;

    if ((state == PTHREAD_CANCEL_ENABLE) && thread->cancelPending) {
        pthread_exit((void *)PTHREAD_CANCELED);
    }

    return (0);
}

/*
 *  ======== pthread_setschedparam ========
 */
int pthread_setschedparam(pthread_t pthread, int policy,
        const struct sched_param *param)
{
    pthread_Obj        *thread = (pthread_Obj *)pthread;
    Task_Handle         task = thread->task;
    UInt                oldPri;
    int                 priority = param->sched_priority;
    UInt                key;
#if ti_sysbios_posix_Settings_supportsMutexPriority__D
    int                 maxPri;
#endif

    if ((priority >= (int)Task_numPriorities) || ((priority == 0)) ||
            (priority < -1)) {
        /* Bad priority value */
        return (EINVAL);
    }

    key = Task_disable();

    oldPri = Task_getPri(task);
    thread->priority = priority;

#if ti_sysbios_posix_Settings_supportsMutexPriority__D
    /*
     *  If the thread is holding a PTHREAD_PRIO_PROTECT or
     *  PTHREAD_PRIO_INHERIT mutex and running at its ceiling, we don't
     *  want to set its priority to a lower value.  Instead, we save the
     *  new priority to set it to, once the mutexes of higher priority
     *  ceilings are released.
     */
    if (!Queue_empty(Queue_handle(&(thread->mutexList)))) {
        maxPri = _pthread_getMaxPrioCeiling(thread);

        if (priority > maxPri) {
            Task_setPri(task, priority);
        }
    }
    else {
        /* The thread owns no mutexes */
        oldPri = Task_setPri(task, priority);
    }
#else
    oldPri = Task_setPri(task, priority);
#endif
    Task_restore(key);

    /* Suppress warning about oldPri not being used. */
    (void)oldPri;

    return (0);
}


/*
 *************************************************************************
 *              internal functions
 *************************************************************************
 */

/*
 *  ======== _pthread_cleanup_pop ========
 */
void _pthread_cleanup_pop(struct _pthread_cleanup_context *context,
        int execute)
{
    pthread_Obj    *thread = (pthread_Obj *)context->thread;

    thread->cleanupList = context->next;

    if (execute) {
        (*(context->fxn))(context->arg);
    }
}

/*
 *  ======== _pthread_cleanup_push ========
 */
void _pthread_cleanup_push(struct _pthread_cleanup_context *context,
        void (*fxn)(void *), void *arg)
{
    pthread_Obj    *thread = (pthread_Obj *)pthread_self();

    context->thread = (pthread_t)thread;
    context->fxn = fxn;
    context->arg = arg;
    context->next = thread->cleanupList;
    thread->cleanupList = context;
}

/*
 *  ======== _pthread_runStub ========
 */
void _pthread_runStub(UArg arg0, UArg arg1)
{
    UInt         key;
    Ptr          arg;
    pthread_Obj *thread = (pthread_Obj *)(xdc_uargToPtr(arg1));

    arg = Task_getEnv(thread->task);
    thread->ret = thread->fxn(arg);

    /* Pop and execute the cleanup handlers */
    while (thread->cleanupList != NULL) {
        _pthread_cleanup_pop(thread->cleanupList, 1);
    }

    /* Cleanup any pthread specific data */
    _pthread_removeThreadKeys((pthread_t)thread);

    key = Task_disable();

    if (!thread->detached) {
        Semaphore_post(Semaphore_handle(&(thread->joinSem)));

        /*
         * Set this task's priority to -1 to prevent it from being put
         * on the terminated queue (and deleted if Task.deleteTerminatedTasks
         * is true). pthread_join() will delete the Task object.
         */
        Task_setPri(thread->task, -1);
        Task_restore(key);
    }
    else {
        Task_restore(key);

        /* Free memory */
#if ti_sysbios_posix_Settings_supportsMutexPriority__D
        Queue_destruct(&(thread->mutexList));
#endif
        Semaphore_destruct(&(thread->joinSem));

        Memory_free(Task_Object_heap(), thread, sizeof(pthread_Obj));

        /* The system will have to clean up the Task object */
    }

    /* Task_exit() is called when returning from this function */
}

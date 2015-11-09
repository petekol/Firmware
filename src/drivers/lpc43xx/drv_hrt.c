/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file drv_hrt.c
 *
 * High-resolution timer callouts and timekeeping.
 *
 * Based on STM32 timer. No PPM
 *
 */

#include <px4_config.h>
#include <nuttx/arch.h>
#include <up_arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/config.h>
#include <chip/lpc43_timer.h>
#include <lpc43_ccu.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <chip.h>

#ifdef HRT_TIMER

#if   HRT_TIMER == 0
# define HRT_TIMER_BASE					LPC43_TIMER0_BASE
# define LPC43M4_IRQ_TIMER				LPC43M4_IRQ_TIMER0
# define LPC43_CCU1_M4_TIMER_CFG		LPC43_CCU1_M4_TIMER0_CFG

#elif HRT_TIMER == 1
# define HRT_TIMER_BASE					LPC43_TIMER1_BASE
# define LPC43M4_IRQ_TIMER				LPC43M4_IRQ_TIMER1
# define LPC43_CCU1_M4_TIMER_CFG		LPC43_CCU1_M4_TIMER1_CFG

#elif HRT_TIMER == 2
# define HRT_TIMER_BASE					LPC43_TIMER2_BASE
# define LPC43M4_IRQ_TIMER				LPC43M4_IRQ_TIMER2
# define LPC43_CCU1_M4_TIMER_CFG		LPC43_CCU1_M4_TIMER2_CFG

#elif HRT_TIMER == 3
# define HRT_TIMER_BASE					LPC43_TIMER3_BASE
# define LPC43M4_IRQ_TIMER				LPC43M4_IRQ_TIMER3
# define LPC43_CCU1_M4_TIMER_CFG		LPC43_CCU1_M4_TIMER3_CFG
#else

# error HRT_TIMER must be a value between 0 and 3
#endif

#if   HRT_TIMER_CHANNEL == 0
#define LPC43_TMR_MR		HRT_TIMER_BASE+LPC43_TMR_MR0_OFFSET
#define TMR_MCR_MRI			TMR_MCR_MR0I
#define TMR_MCR_MRR			TMR_MCR_MR0R
#define TMR_MCR_MRS			TMR_MCR_MR0S
#define TMR_IR_MR			 TMR_IR_MR0

#elif HRT_TIMER_CHANNEL == 1
#define LPC43_TMR_MR		HRT_TIMER_BASE+LPC43_TMR_MR1_OFFSET
#define TMR_MCR_MRI			TMR_MCR_MR1I
#define TMR_MCR_MRR			TMR_MCR_MR1R
#define TMR_MCR_MRS			TMR_MCR_MR1S
#define TMR_IR_MR			 TMR_IR_MR1

#elif HRT_TIMER_CHANNEL == 2
#define LPC43_TMR_MR		HRT_TIMER_BASE+LPC43_TMR_MR2_OFFSET
#define TMR_MCR_MRI			TMR_MCR_MR2I
#define TMR_MCR_MRR			TMR_MCR_MR2R
#define TMR_MCR_MRS			TMR_MCR_MR2S
#define TMR_IR_MR			 TMR_IR_MR2

#elif HRT_TIMER_CHANNEL == 3
#define LPC43_TMR_MR		HRT_TIMER_BASE+LPC43_TMR_MR3_OFFSET
#define TMR_MCR_MRI			TMR_MCR_MR3I
#define TMR_MCR_MRR			TMR_MCR_MR3R
#define TMR_MCR_MRS			TMR_MCR_MR3S
#define TMR_IR_MR			 TMR_IR_MR3
#else
# error HRT_TIMER_CHANNEL must be a value between 0 and 3
#endif

/*
 * HRT clock must be a multiple of 1MHz greater than 1MHz
 */
#if (LPC43_CCLK % USEC_PER_SEC) != 0
# error HRT_TIMER_CLOCK must be a multiple of 1MHz
#endif
#if LPC43_CCLK <= USEC_PER_SEC
# error HRT_TIMER_CLOCK must be greater than 1MHz
#endif

/**
 * Minimum/maximum deadlines.
 *
 * These are suitable for use with a 32-bit timer/counter clocked
 * at 1MHz.  The high-resolution timer need only guarantee that it
 * not wrap more than once in the 50ms period for absolute time to
 * be consistently maintained.
 *
 * The minimum deadline must be such that the time taken between
 * reading a time and writing a deadline to the timer cannot
 * result in missing the deadline.
 */
#define HRT_INTERVAL_MIN	50
#define HRT_INTERVAL_MAX	(UINT32_MAX/8*6)

/*
 * Period of the free-running counter, in microseconds.
 */
#define HRT_COUNTER_PERIOD	UINT32_MAX

/*
 * Scaling factor(s) for the free-running counter; convert an input
 * in counts to a time in microseconds.
 */
#define HRT_COUNTER_SCALE(_c)	(_c)

/*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;

/* latency baseline (last compare value applied) */
static uint16_t			latency_baseline;

/* timer count at interrupt (for latency purposes) */
static uint16_t			latency_actual;

/* latency histogram */
#define LATENCY_BUCKET_COUNT 8
__EXPORT const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
__EXPORT const uint16_t	latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t		latency_counters[LATENCY_BUCKET_COUNT + 1];


/* timer-specific functions */
static void		hrt_tim_init(void);
static int		hrt_tim_isr(int irq, void *context);
static void		hrt_latency_update(void);

/* callout list manipulation */
static void		hrt_call_internal(struct hrt_call *entry,
		hrt_abstime deadline,
		hrt_abstime interval,
		hrt_callout callout,
		void *arg);
static void		hrt_call_enter(struct hrt_call *entry);
static void		hrt_call_reschedule(void);
static void		hrt_call_invoke(void);

/**
 * Initialise the timer we are going to use.
 *
 * We expect that we'll own one of the reduced-function STM32 general
 * timers, and that we can use channel 1 in compare mode.
 */
static void
hrt_tim_init(void)
{

	irqstate_t flags = irqsave();

	/* power on */
	uint32_t regval  = getreg32(LPC43_CCU1_M4_TIMER_CFG);
	regval |= CCU_CLK_CFG_RUN;
	putreg32(regval, LPC43_CCU1_M4_TIMER_CFG);


	putreg32(0, HRT_TIMER_BASE+LPC43_TMR_TCR_OFFSET); /* disable */

	putreg32(LPC43_CCLK/USEC_PER_SEC-1, HRT_TIMER_BASE+LPC43_TMR_PR_OFFSET); /* set clock */

	putreg32(0, HRT_TIMER_BASE+LPC43_TMR_PC_OFFSET); /* reset prescale counter */
	putreg32(0, HRT_TIMER_BASE+LPC43_TMR_TC_OFFSET); /* reset timer counter */

	putreg32(TMR_MCR_MRI, HRT_TIMER_BASE+LPC43_TMR_MCR_OFFSET); /* only interrupt, no stop and reset */

	putreg32(0, HRT_TIMER_BASE+LPC43_TMR_CCR_OFFSET); /* do not use capture */
	putreg32(0, HRT_TIMER_BASE+LPC43_TMR_CTCR_OFFSET); /* counter/timer mode */


	putreg32(HRT_COUNTER_PERIOD/2, LPC43_TMR_MR); /* set an initial match a little ways off */

	putreg32(TMR_IR_MR0|TMR_IR_MR1|TMR_IR_MR2|TMR_IR_MR3|TMR_IR_CR0|TMR_IR_CR1|TMR_IR_CR2|TMR_IR_CR3,
			HRT_TIMER_BASE + LPC43_TMR_IR_OFFSET); /* clear pending interrupts */

	irq_attach(LPC43M4_IRQ_TIMER, hrt_tim_isr); /* claim our interrupt vector */
	up_enable_irq(LPC43M4_IRQ_TIMER); /* enable interrupts */


	putreg32(TMR_TCR_EN, HRT_TIMER_BASE+LPC43_TMR_TCR_OFFSET); /* enable the timer */

	irqrestore(flags);
}

/**
 * Handle the compare interupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int
hrt_tim_isr(int irq, void *context)
{
	uint32_t status;

	/* grab the timer for latency tracking purposes */
	latency_actual = getreg32(HRT_TIMER_BASE+LPC43_TMR_TC_OFFSET);

	/* copy interrupt status */
	status = getreg32(HRT_TIMER_BASE+LPC43_TMR_IR_OFFSET);

	/* ack the interrupts we just read */
	putreg32(HRT_TIMER_BASE+LPC43_TMR_IR_OFFSET,status);

	/* was this a timer tick? */
	if (status & TMR_IR_MR) {

		/* do latency calculations */
		hrt_latency_update();

		/* run any callouts that have met their deadline */
		hrt_call_invoke();

		/* and schedule the next interrupt */
		hrt_call_reschedule();
	}

	return OK;
}

/**
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime
hrt_absolute_time(void)
{
	hrt_abstime	abstime;
	uint32_t	count;
	irqstate_t	flags;

	/*
	 * Counter state.  Marked volatile as they may change
	 * inside this routine but outside the irqsave/restore
	 * pair.  Discourage the compiler from moving loads/stores
	 * to these outside of the protected range.
	 */
	static volatile hrt_abstime base_time;
	static volatile uint32_t last_count;

	/* prevent re-entry */
	flags = irqsave();

	/* get the current counter value */
	count = getreg32(HRT_TIMER_BASE+LPC43_TMR_TC_OFFSET);

	/*
	 * Determine whether the counter has wrapped since the
	 * last time we're called.
	 *
	 * This simple test is sufficient due to the guarantee that
	 * we are always called at least once per counter period.
	 */
	if (count < last_count)
		base_time += HRT_COUNTER_PERIOD;

	/* save the count for next time */
	last_count = count;

	/* compute the current time */
	abstime = HRT_COUNTER_SCALE(base_time + count);

	irqrestore(flags);

	return abstime;
}

/**
 * Convert a timespec to absolute time
 */
hrt_abstime
ts_to_abstime(struct timespec *ts)
{
	hrt_abstime	result;

	result = (hrt_abstime)(ts->tv_sec) * USEC_PER_SEC;
	result += ts->tv_nsec / NSEC_PER_USEC;

	return result;
}

/**
 * Convert absolute time to a timespec.
 */
void
abstime_to_ts(struct timespec *ts, hrt_abstime abstime)
{
	ts->tv_sec = abstime / USEC_PER_SEC;
	abstime -= ts->tv_sec * USEC_PER_SEC;
	ts->tv_nsec = abstime * NSEC_PER_USEC;
}

/**
 * Compare a time value with the current time.
 */
hrt_abstime
hrt_elapsed_time(const volatile hrt_abstime *then)
{
	irqstate_t flags = irqsave();

	hrt_abstime delta = hrt_absolute_time() - *then;

	irqrestore(flags);

	return delta;
}

/**
 * Store the absolute time in an interrupt-safe fashion
 */
hrt_abstime
hrt_store_absolute_time(volatile hrt_abstime *now)
{
	irqstate_t flags = irqsave();

	hrt_abstime ts = hrt_absolute_time();

	irqrestore(flags);

	return ts;
}

/**
 * Initalise the high-resolution timing module.
 */
void
hrt_init(void)
{
	sq_init(&callout_queue);
	hrt_tim_init();

}

/**
 * Call callout(arg) after interval has elapsed.
 */
void
hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  0,
			  callout,
			  arg);
}

/**
 * Call callout(arg) at calltime.
 */
void
hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

/**
 * Call callout(arg) every period.
 */
void
hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

static void
hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	irqstate_t flags = irqsave();

	/* if the entry is currently queued, remove it */
	/* note that we are using a potentially uninitialised
	   entry->link here, but it is safe as sq_rem() doesn't
	   dereference the passed node unless it is found in the
	   list. So we potentially waste a bit of time searching the
	   queue for the uninitialised entry->link but we don't do
	   anything actually unsafe.
	*/
	if (entry->deadline != 0)
		sq_rem(&entry->link, &callout_queue);

	entry->deadline = deadline;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);

	irqrestore(flags);
}

/**
 * If this returns true, the call has been invoked and removed from the callout list.
 *
 * Always returns false for repeating callouts.
 */
bool
hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/**
 * Remove the entry from the callout list.
 */
void
hrt_cancel(struct hrt_call *entry)
{
	irqstate_t flags = irqsave();

	sq_rem(&entry->link, &callout_queue);
	entry->deadline = 0;

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;

	irqrestore(flags);
}

static void
hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call, *next;

	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);
		//lldbg("call enter at head, reschedule\n");
		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();

	} else {
		do {
			next = (struct hrt_call *)sq_next(&call->link);

			if ((next == NULL) || (entry->deadline < next->deadline)) {
				//lldbg("call enter after head\n");
				sq_addafter(&call->link, &entry->link, &callout_queue);
				break;
			}
		} while ((call = next) != NULL);
	}

	//lldbg("scheduled\n");
}

static void
hrt_call_invoke(void)
{
	struct hrt_call	*call;
	hrt_abstime deadline;

	while (true) {
		/* get the current time */
		hrt_abstime now = hrt_absolute_time();

		call = (struct hrt_call *)sq_peek(&callout_queue);

		if (call == NULL)
			break;

		if (call->deadline > now)
			break;

		sq_rem(&call->link, &callout_queue);
		//lldbg("call pop\n");

		/* save the intended deadline for periodic calls */
		deadline = call->deadline;

		/* zero the deadline, as the call has occurred */
		call->deadline = 0;

		/* invoke the callout (if there is one) */
		if (call->callout) {
			//lldbg("call %p: %p(%p)\n", call, call->callout, call->arg);
			call->callout(call->arg);
		}

		/* if the callout has a non-zero period, it has to be re-entered */
		if (call->period != 0) {
			// re-check call->deadline to allow for
			// callouts to re-schedule themselves
			// using hrt_call_delay()
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
			}

			hrt_call_enter(call);
		}
	}
}

/**
 * Reschedule the next timer interrupt.
 *
 * This routine must be called with interrupts disabled.
 */
static void
hrt_call_reschedule()
{
	hrt_abstime	now = hrt_absolute_time();
	struct hrt_call	*next = (struct hrt_call *)sq_peek(&callout_queue);
	hrt_abstime	deadline = now + HRT_INTERVAL_MAX;

	/*
	 * Determine what the next deadline will be.
	 *
	 * Note that we ensure that this will be within the counter
	 * period, so that when we truncate all but the low 32 bits
	 * the next time the compare matches it will be the deadline
	 * we want.
	 *
	 * It is important for accurate timekeeping that the compare
	 * interrupt fires sufficiently often that the base_time update in
	 * hrt_absolute_time runs at least once per timer period.
	 */
	if (next != NULL) {
		//lldbg("entry in queue\n");
		if (next->deadline <= (now + HRT_INTERVAL_MIN)) {
			//lldbg("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			deadline = now + HRT_INTERVAL_MIN;

		} else if (next->deadline < deadline) {
			//lldbg("due soon\n");
			deadline = next->deadline;
		}
	}

	//lldbg("schedule for %u at %u\n", (unsigned)(deadline & 0xffffffff), (unsigned)(now & 0xffffffff));

	/* set the new compare value and remember it for latency tracking */
	latency_baseline = deadline & HRT_COUNTER_PERIOD;

	putreg32(latency_baseline, LPC43_TMR_MR);
}

static void
hrt_latency_update(void)
{
	uint16_t latency = latency_actual - latency_baseline;
	unsigned	index;

	/* bounded buckets */
	for (index = 0; index < LATENCY_BUCKET_COUNT; index++) {
		if (latency <= latency_buckets[index]) {
			latency_counters[index]++;
			return;
		}
	}

	/* catch-all at the end */
	latency_counters[index]++;
}

void
hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void
hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}

#endif /* HRT_TIMER */

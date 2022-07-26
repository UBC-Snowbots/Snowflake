/*-
 * Copyright (c) 1991, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)queue.h	8.5 (Berkeley) 8/20/94
 * $FreeBSD: src/sys/sys/queue.h,v 1.60.2.1 2005/08/16 22:41:39 phk Exp $
 */

#ifndef _MOS_QUEUE_H_
#define	_MOS_QUEUE_H_

/*
 * This file defines four types of data structures: singly-linked lists,
 * singly-linked tail queues, lists and tail queues.
 *
 * A singly-linked list is headed by a single forward pointer. The elements
 * are singly linked for minimum space and pointer manipulation overhead at
 * the expense of O(n) removal for arbitrary elements. New elements can be
 * added to the list after an existing element or at the head of the list.
 * Elements being removed from the head of the list should use the explicit
 * macro for this purpose for optimum efficiency. A singly-linked list may
 * only be traversed in the forward direction.  Singly-linked lists are ideal
 * for applications with large datasets and few or no removals or for
 * implementing a LIFO queue.
 *
 * A singly-linked tail queue is headed by a pair of pointers, one to the
 * head of the list and the other to the tail of the list. The elements are
 * singly linked for minimum space and pointer manipulation overhead at the
 * expense of O(n) removal for arbitrary elements. New elements can be added
 * to the list after an existing element, at the head of the list, or at the
 * end of the list. Elements being removed from the head of the tail queue
 * should use the explicit macro for this purpose for optimum efficiency.
 * A singly-linked tail queue may only be traversed in the forward direction.
 * Singly-linked tail queues are ideal for applications with large datasets
 * and few or no removals or for implementing a FIFO queue.
 *
 * A list is headed by a single forward pointer (or an array of forward
 * pointers for a hash table header). The elements are doubly linked
 * so that an arbitrary element can be removed without a need to
 * traverse the list. New elements can be added to the list before
 * or after an existing element or at the head of the list. A list
 * may only be traversed in the forward direction.
 *
 * A tail queue is headed by a pair of pointers, one to the head of the
 * list and the other to the tail of the list. The elements are doubly
 * linked so that an arbitrary element can be removed without a need to
 * traverse the list. New elements can be added to the list before or
 * after an existing element, at the head of the list, or at the end of
 * the list. A tail queue may be traversed in either direction.
 *
 * For details on the use of these macros, see the queue(3) manual page.
 *
 *
 *								MSLIST	MLIST	MSMTAILQ MTAILQ
 * _HEAD						+		+		+		+
 * _HEAD_INITIALIZER			+		+		+		+
 * _ENTRY						+		+		+		+
 * _INIT						+		+		+		+
 * _EMPTY						+		+		+		+
 * _FIRST						+		+		+		+
 * _NEXT						+		+		+		+
 * _PREV						-		-		-		+
 * _LAST						-		-		+		+
 * _FOREACH						+		+		+		+
 * _FOREACH_SAFE				+		+		+		+
 * _FOREACH_REVERSE				-		-		-		+
 * _FOREACH_REVERSE_SAFE		-		-		-		+
 * _INSERT_HEAD					+		+		+		+
 * _INSERT_BEFORE				-		+		-		+
 * _INSERT_AFTER				+		+		+		+
 * _INSERT_TAIL					-		-		+		+
 * _CONCAT						-		-		+		+
 * _REMOVE_HEAD					+		-		+		-
 * _REMOVE						+		+		+		+
 *
 */

#if QUEUE_MACRO_DEBUG
/* Store the last 2 places the queue element or head was altered */
struct qm_trace {
	const char *lastfile;
	int lastline;
	const char *prevfile;
	int prevline;
};

#define	TRACEBUF	struct qm_trace trace;
#define	TRASHIT(x)	do {(x) = (void *)-1;} while (0)

#define	MQMD_TRACE_HEAD(head) do {					\
	(head)->trace.prevline = (head)->trace.lastline;		\
	(head)->trace.prevfile = (head)->trace.lastfile;		\
	(head)->trace.lastline = __LINE__;				\
	(head)->trace.lastfile = __FILE__;				\
} while (0)

#define	MQMD_TRACE_ELEM(elem) do {					\
	(elem)->trace.prevline = (elem)->trace.lastline;		\
	(elem)->trace.prevfile = (elem)->trace.lastfile;		\
	(elem)->trace.lastline = __LINE__;				\
	(elem)->trace.lastfile = __FILE__;				\
} while (0)

#else

#define	MQMD_TRACE_ELEM(elem)
#define	MQMD_TRACE_HEAD(head)
#define	TRACEBUF
#define	TRASHIT(x)
#endif	/* QUEUE_MACRO_DEBUG */

/*
 * Singly-linked List declarations.
 */
#define	MSLIST_HEAD(name, type)			\
struct name {								\
	struct type *slh_first;	/* first element */			\
}

#define	MSLIST_HEAD_INITIALIZER(head)					\
	{ NULL }

#define	MSLIST_ENTRY(type)						\
struct {											\
	struct type *sle_next;	/* next element */		\
}

/*
 * Singly-linked List functions.
 */
#define	MSLIST_EMPTY(head)	((head)->slh_first == NULL)

#define	MSLIST_FIRST(head)	((head)->slh_first)

#define	MSLIST_FOREACH(var, head, field)					\
	for ((var) = MSLIST_FIRST((head));				\
	    (var);							\
	    (var) = MSLIST_NEXT((var), field))

#define	MSLIST_FOREACH_SAFE(var, head, field, tvar)			\
	for ((var) = MSLIST_FIRST((head));				\
	    (var) && ((tvar) = MSLIST_NEXT((var), field), 1);		\
	    (var) = (tvar))

#define	MSLIST_FOREACH_PREVPTR(var, varp, head, field)			\
	for ((varp) = &MSLIST_FIRST((head));				\
	    ((var) = *(varp)) != NULL;					\
	    (varp) = &MSLIST_NEXT((var), field))

#define	MSLIST_INIT(head) do {						\
	MSLIST_FIRST((head)) = NULL;					\
} while (0)

#define	MSLIST_INSERT_AFTER(slistelm, elm, field) do {			\
	MSLIST_NEXT((elm), field) = MSLIST_NEXT((slistelm), field);	\
	MSLIST_NEXT((slistelm), field) = (elm);				\
} while (0)

#define	MSLIST_INSERT_HEAD(head, elm, field) do {			\
	MSLIST_NEXT((elm), field) = MSLIST_FIRST((head));			\
	MSLIST_FIRST((head)) = (elm);					\
} while (0)

#define	MSLIST_NEXT(elm, field)	((elm)->field.sle_next)

#define	MSLIST_REMOVE(head, elm, type, field) do {			\
	if (MSLIST_FIRST((head)) == (elm)) {				\
		MSLIST_REMOVE_HEAD((head), field);			\
	}								\
	else {								\
		struct type *curelm = MSLIST_FIRST((head));		\
		while (MSLIST_NEXT(curelm, field) != (elm))		\
			curelm = MSLIST_NEXT(curelm, field);		\
		MSLIST_NEXT(curelm, field) =				\
		    MSLIST_NEXT(MSLIST_NEXT(curelm, field), field);	\
	}								\
} while (0)

#define	MSLIST_REMOVE_HEAD(head, field) do {				\
	MSLIST_FIRST((head)) = MSLIST_NEXT(MSLIST_FIRST((head)), \
	field);	\
} while (0)

/*
 * Singly-linked Tail queue declarations.
 */
#define	MSMTAILQ_HEAD(name, type)						\
struct name {								\
	struct type *stqh_first;/* first element */			\
	struct type **stqh_last;/* addr of last next element */		\
}

#define	MSMTAILQ_HEAD_INITIALIZER(head)					\
	{ NULL, &(head).stqh_first }

#define	MSMTAILQ_ENTRY(type)						\
struct {								\
	struct type *stqe_next;	/* next element */			\
}

/*
 * Singly-linked Tail queue functions.
 */
#define	MSMTAILQ_CONCAT(head1, head2) do {				\
	if (!MSMTAILQ_EMPTY((head2))) {					\
		*(head1)->stqh_last = (head2)->stqh_first;		\
		(head1)->stqh_last = (head2)->stqh_last;		\
		MSMTAILQ_INIT((head2));					\
	}								\
} while (0)

#define	MSMTAILQ_EMPTY(head)	((head)->stqh_first == NULL)

#define	MSMTAILQ_FIRST(head)	((head)->stqh_first)

#define	MSMTAILQ_FOREACH(var, head, field)				\
	for((var) = MSMTAILQ_FIRST((head));				\
	   (var);							\
	   (var) = MSMTAILQ_NEXT((var), field))


#define	MSMTAILQ_FOREACH_SAFE(var, head, field, tvar)			\
	for ((var) = MSMTAILQ_FIRST((head));				\
	    (var) && ((tvar) = MSMTAILQ_NEXT((var), field), 1);		\
	    (var) = (tvar))

#define	MSMTAILQ_INIT(head) do {						\
	MSMTAILQ_FIRST((head)) = NULL;					\
	(head)->stqh_last = &MSMTAILQ_FIRST((head));			\
} while (0)

#define	MSMTAILQ_INSERT_AFTER(head, tqelm, elm, field) do {		\
	if ((MSMTAILQ_NEXT((elm), field) = MSMTAILQ_NEXT((tqelm), field)) == NULL)\
		(head)->stqh_last = &MSMTAILQ_NEXT((elm), field);		\
	MSMTAILQ_NEXT((tqelm), field) = (elm);				\
} while (0)

#define	MSMTAILQ_INSERT_HEAD(head, elm, field) do {			\
	if ((MSMTAILQ_NEXT((elm), field) = MSMTAILQ_FIRST((head))) == NULL)	\
		(head)->stqh_last = &MSMTAILQ_NEXT((elm), field);		\
	MSMTAILQ_FIRST((head)) = (elm);					\
} while (0)

#define	MSMTAILQ_INSERT_TAIL(head, elm, field) do {			\
	MSMTAILQ_NEXT((elm), field) = NULL;				\
	*(head)->stqh_last = (elm);					\
	(head)->stqh_last = &MSMTAILQ_NEXT((elm), field);			\
} while (0)

#define	MSMTAILQ_LAST(head, type, field)					\
	(MSMTAILQ_EMPTY((head)) ?						\
		NULL :							\
	        ((struct type *)					\
		((char *)((head)->stqh_last) - __offsetof(struct type, field))))

#define	MSMTAILQ_NEXT(elm, field)	((elm)->field.stqe_next)

#define	MSMTAILQ_REMOVE(head, elm, type, field) do {			\
	if (MSMTAILQ_FIRST((head)) == (elm)) {				\
		MSMTAILQ_REMOVE_HEAD((head), field);			\
	}								\
	else {								\
		struct type *curelm = MSMTAILQ_FIRST((head));		\
		while (MSMTAILQ_NEXT(curelm, field) != (elm))		\
			curelm = MSMTAILQ_NEXT(curelm, field);		\
		if ((MSMTAILQ_NEXT(curelm, field) =			\
		     MSMTAILQ_NEXT(MSMTAILQ_NEXT(curelm, field), field)) == NULL)\
			(head)->stqh_last = &MSMTAILQ_NEXT((curelm), field);\
	}								\
} while (0)

#define	MSMTAILQ_REMOVE_HEAD(head, field) do {				\
	if ((MSMTAILQ_FIRST((head)) =					\
	     MSMTAILQ_NEXT(MSMTAILQ_FIRST((head)), field)) == NULL)		\
		(head)->stqh_last = &MSMTAILQ_FIRST((head));		\
} while (0)

#define	MSMTAILQ_REMOVE_HEAD_UNTIL(head, elm, field) do {			\
	if ((MSMTAILQ_FIRST((head)) = MSMTAILQ_NEXT((elm), field)) == NULL)	\
		(head)->stqh_last = &MSMTAILQ_FIRST((head));		\
} while (0)

/*
 * List declarations.
 */
#define	MLIST_HEAD(name, type)						\
struct name {								\
	struct type *lh_first;	/* first element */			\
}

#define	MLIST_HEAD_INITIALIZER(head)					\
	{ NULL }

#define	MLIST_ENTRY(type)						\
struct {								\
	struct type *le_next;	/* next element */			\
	struct type **le_prev;	/* address of previous next element */	\
}

/*
 * List functions.
 */

#define	MLIST_EMPTY(head)	((head)->lh_first == NULL)

#define	MLIST_FIRST(head)	((head)->lh_first)

#define	MLIST_FOREACH(var, head, field)					\
	for ((var) = MLIST_FIRST((head));				\
	    (var);							\
	    (var) = MLIST_NEXT((var), field))

#define	MLIST_FOREACH_SAFE(var, head, field, tvar)			\
	for ((var) = MLIST_FIRST((head));				\
	    (var) && ((tvar) = MLIST_NEXT((var), field), 1);		\
	    (var) = (tvar))

#define	MLIST_INIT(head) do {						\
	MLIST_FIRST((head)) = NULL;					\
} while (0)

#define	MLIST_INSERT_AFTER(listelm, elm, field) do {			\
	if ((MLIST_NEXT((elm), field) = MLIST_NEXT((listelm), field)) != NULL)\
		MLIST_NEXT((listelm), field)->field.le_prev =		\
		    &MLIST_NEXT((elm), field);				\
	MLIST_NEXT((listelm), field) = (elm);				\
	(elm)->field.le_prev = &MLIST_NEXT((listelm), field);		\
} while (0)

#define	MLIST_INSERT_BEFORE(listelm, elm, field) do {			\
	(elm)->field.le_prev = (listelm)->field.le_prev;		\
	MLIST_NEXT((elm), field) = (listelm);				\
	*(listelm)->field.le_prev = (elm);				\
	(listelm)->field.le_prev = &MLIST_NEXT((elm), field);		\
} while (0)

#define	MLIST_INSERT_HEAD(head, elm, field) do {				\
	if ((MLIST_NEXT((elm), field) = MLIST_FIRST((head))) != NULL)	\
		MLIST_FIRST((head))->field.le_prev = &MLIST_NEXT((elm), field);\
	MLIST_FIRST((head)) = (elm);					\
	(elm)->field.le_prev = &MLIST_FIRST((head));			\
} while (0)

#define	MLIST_NEXT(elm, field)	((elm)->field.le_next)

#define	MLIST_REMOVE(elm, field) do {					\
	if (MLIST_NEXT((elm), field) != NULL)				\
		MLIST_NEXT((elm), field)->field.le_prev = 		\
		    (elm)->field.le_prev;				\
	*(elm)->field.le_prev = MLIST_NEXT((elm), field);		\
} while (0)

/*
 * Tail queue declarations.
 */
#define	MTAILQ_HEAD(name, type)						\
struct name {								\
	struct type *tqh_first;	/* first element */			\
	struct type **tqh_last;	/* addr of last next element */		\
	TRACEBUF							\
}

#define	MTAILQ_HEAD_INITIALIZER(head)					\
	{ NULL, &(head).tqh_first }

#define	MTAILQ_ENTRY(type)						\
struct {								\
	struct type *tqe_next;	/* next element */			\
	struct type **tqe_prev;	/* address of previous next element */	\
	TRACEBUF							\
}

/*
 * Tail queue functions.
 */
#define	MTAILQ_CONCAT(head1, head2, field) do {				\
	if (!MTAILQ_EMPTY(head2)) {					\
		*(head1)->tqh_last = (head2)->tqh_first;		\
		(head2)->tqh_first->field.tqe_prev = (head1)->tqh_last;	\
		(head1)->tqh_last = (head2)->tqh_last;			\
		MTAILQ_INIT((head2));					\
		MQMD_TRACE_HEAD(head1);					\
		MQMD_TRACE_HEAD(head2);					\
	}								\
} while (0)

#define	MTAILQ_EMPTY(head)	((head)->tqh_first == NULL)

#define	MTAILQ_FIRST(head)	((head)->tqh_first)

#define	MTAILQ_FOREACH(var, head, field)					\
	for ((var) = MTAILQ_FIRST((head));				\
	    (var);							\
	    (var) = MTAILQ_NEXT((var), field))

#define	MTAILQ_FOREACH_SAFE(var, head, field, tvar)			\
	for ((var) = MTAILQ_FIRST((head));				\
	    (var) && ((tvar) = MTAILQ_NEXT((var), field), 1);		\
	    (var) = (tvar))

#define	MTAILQ_FOREACH_REVERSE(var, head, headname, field)		\
	for ((var) = MTAILQ_LAST((head), headname);			\
	    (var);							\
	    (var) = MTAILQ_PREV((var), headname, field))

#define	MTAILQ_FOREACH_REVERSE_SAFE(var, head, headname, field, tvar)	\
	for ((var) = MTAILQ_LAST((head), headname);			\
	    (var) && ((tvar) = MTAILQ_PREV((var), headname, field), 1);	\
	    (var) = (tvar))

#define	MTAILQ_INIT(head) do {						\
	MTAILQ_FIRST((head)) = NULL;					\
	(head)->tqh_last = &MTAILQ_FIRST((head));			\
	MQMD_TRACE_HEAD(head);						\
} while (0)

#define	MTAILQ_INSERT_AFTER(head, listelm, elm, field) do {		\
	if ((MTAILQ_NEXT((elm), field) = MTAILQ_NEXT((listelm), field)) != NULL)\
		MTAILQ_NEXT((elm), field)->field.tqe_prev = 		\
		    &MTAILQ_NEXT((elm), field);				\
	else {								\
		(head)->tqh_last = &MTAILQ_NEXT((elm), field);		\
		MQMD_TRACE_HEAD(head);					\
	}								\
	MTAILQ_NEXT((listelm), field) = (elm);				\
	(elm)->field.tqe_prev = &MTAILQ_NEXT((listelm), field);		\
	MQMD_TRACE_ELEM(&(elm)->field);					\
	MQMD_TRACE_ELEM(&listelm->field);				\
} while (0)

#define	MTAILQ_INSERT_BEFORE(listelm, elm, field) do {			\
	(elm)->field.tqe_prev = (listelm)->field.tqe_prev;		\
	MTAILQ_NEXT((elm), field) = (listelm);				\
	*(listelm)->field.tqe_prev = (elm);				\
	(listelm)->field.tqe_prev = &MTAILQ_NEXT((elm), field);		\
	MQMD_TRACE_ELEM(&(elm)->field);					\
	MQMD_TRACE_ELEM(&listelm->field);				\
} while (0)

#define	MTAILQ_INSERT_HEAD(head, elm, field) do {			\
	if ((MTAILQ_NEXT((elm), field) = MTAILQ_FIRST((head))) != NULL)	\
		MTAILQ_FIRST((head))->field.tqe_prev =			\
		    &MTAILQ_NEXT((elm), field);				\
	else								\
		(head)->tqh_last = &MTAILQ_NEXT((elm), field);		\
	MTAILQ_FIRST((head)) = (elm);					\
	(elm)->field.tqe_prev = &MTAILQ_FIRST((head));			\
	MQMD_TRACE_HEAD(head);						\
	MQMD_TRACE_ELEM(&(elm)->field);					\
} while (0)

#define	MTAILQ_INSERT_TAIL(head, elm, field) do {			\
	MTAILQ_NEXT((elm), field) = NULL;				\
	(elm)->field.tqe_prev = (head)->tqh_last;			\
	*(head)->tqh_last = (elm);					\
	(head)->tqh_last = &MTAILQ_NEXT((elm), field);			\
	MQMD_TRACE_HEAD(head);						\
	MQMD_TRACE_ELEM(&(elm)->field);					\
} while (0)

#define	MTAILQ_LAST(head, headname)					\
	(*(((struct headname *)((head)->tqh_last))->tqh_last))

#define	MTAILQ_NEXT(elm, field) ((elm)->field.tqe_next)

#define	MTAILQ_PREV(elm, headname, field)				\
	(*(((struct headname *)((elm)->field.tqe_prev))->tqh_last))

#define	MTAILQ_REMOVE(head, elm, field) do {				\
	if ((MTAILQ_NEXT((elm), field)) != NULL)				\
		MTAILQ_NEXT((elm), field)->field.tqe_prev = 		\
		    (elm)->field.tqe_prev;				\
	else {								\
		(head)->tqh_last = (elm)->field.tqe_prev;		\
		MQMD_TRACE_HEAD(head);					\
	}								\
	*(elm)->field.tqe_prev = MTAILQ_NEXT((elm), field);		\
	TRASHIT((elm)->field.tqe_next);					\
	TRASHIT((elm)->field.tqe_prev);					\
	MQMD_TRACE_ELEM(&(elm)->field);					\
} while (0)


#endif /* !_MOS_QUEUE_H_ */

#include "mos_basic.h"
#include "mos_os.h"
#include "mos_assert.h"
#include "mos_macrocompat.h"

void _mos_malloc_init(void);
void _mos_malloc_fini(void);

#if defined(MOS_TRACK_ALLOCATIONS)

#include <sys/types.h>
#if !defined(_KERNEL)
# include <stdio.h>
#endif

#include "mos_stacktrace.h"
#include "mos_assert.h"
#include "mos_lock.h"
#include "bsdqueue.h"
#include "bsdtree.h"

#define MOS_MAX_FREELISTSZ		32
#define BACKTRACE_SIZE			32

/*
 * Current allocation set flags.
 *
 * These are used to breakout allocation for use in finding leaks
 * in phases.
 */
static uint32_t	malloc_set;

struct mos_allocation {
	size_t						ma_sz;		/* size of the allocation */
	void						*ma_ptr;	/* address of allocation */
	uint32_t					ma_set;		/* allocation set */
#if defined(MOS_TRACK_ALLOCATION_DETAILS)
	int							ma_flags;	/* allocation flags */
	int							ma_line;
	const char 					*ma_func;
	const char 					*ma_file;
	void						*ma_backtrace[BACKTRACE_SIZE];
	size_t						ma_backtracesz;
#endif
	RB_ENTRY(mos_allocation)	ma_tlink;	/* RB Tree linkage */
	MTAILQ_ENTRY(mos_allocation)	ma_llink;	/* List linkage */
};

static int
mos_addr_cmp(struct mos_allocation *m1, struct mos_allocation *m2) {

	if ((size_t)m1->ma_ptr < (size_t)m2->ma_ptr)
		return (-1);
	if ((size_t)m1->ma_ptr == (size_t)m2->ma_ptr)
		return (0);
	return (1);
}

typedef RB_HEAD(mos_allocation_tree, mos_allocation) mos_allocation_tree_t;
RB_PROTOTYPE(mos_allocation_tree, mos_allocation, ma_tlink, mos_addr_cmp)
RB_GENERATE(mos_allocation_tree, mos_allocation, ma_tlink, mos_addr_cmp)

typedef MTAILQ_HEAD(mos_allocation_list, mos_allocation) mos_allocation_list_t;

static mos_allocation_list_t	freelist;
static mos_allocation_tree_t	tree128;
static mos_allocation_tree_t	tree1024;
static mos_allocation_tree_t	treeBIG;

static mos_mutex_t				freelistlk;
static mos_mutex_t				tree128lk;
static mos_mutex_t				tree1024lk;
static mos_mutex_t				treeBIGlk;

static size_t					freelistsz;
static ssize_t					tree128sz;
static ssize_t					tree1024sz;
static ssize_t					treeBIGsz;
static size_t					metaallocs;

#if defined(MOS_TRACK_ALLOCATION_DETAILS)
#define TRACK_BACKTRACE
#endif

#if defined(TRACK_BACKTRACE)
static void
print_backtrace(mos_malloc_printer_t ptr, void *arg, struct mos_allocation *ma) {
	char symname[MOS_PATH_MAX];
	size_t i;

	for (i = 0; i < ma->ma_backtracesz; i++) {
		if (mos_getsymbolname(ma->ma_backtrace[i], symname, sizeof (symname)) != 0)
			mos_strlcpy(symname, "unknown", sizeof (symname));

		ptr(arg, "  %s\n", symname);
	}
}
#endif

#if defined(MOS_TRACK_ALLOCATION_DETAILS)
static void
get_backtrace(struct mos_allocation *ma) {

	ma->ma_backtracesz = mos_stacktrace(ma->ma_backtrace, BACKTRACE_SIZE);
}
#endif

static void
dump_allocation(mos_allocation_tree_t *tree, mos_mutex_t *lk,
  mos_malloc_printer_t ptr, void *arg, uint32_t set, const char *msg) {
	struct mos_allocation *ma;
	size_t acnt;
	uint8_t *p;
	size_t sz;
	size_t i;

	acnt = 0;
	sz = 0;

	mos_mutex_lock(lk);

	ptr(arg, "\n%s\n", msg);
	ptr(arg, "----------------------------------------------------------\n");
	RB_FOREACH(ma, mos_allocation_tree, tree) {
		if (set != 0 && (ma->ma_set & set) != set)
			continue;
		ptr(arg, "\n%p (%u) <0x%x>\n [", (void *)ma->ma_ptr, (unsigned)ma->ma_sz, ma->ma_set);
		p = ma->ma_ptr;
		for (i = 0; i < ma->ma_sz && i < 12; i++) {
			if (mos_isprint(p[i]))
				ptr(arg, "%c", p[i]);
			else
				ptr(arg, "(0x%x)", (uint32_t)p[i]);
		}
		if (i == 12)
			ptr(arg, "...");
		ptr(arg, "]\n");

#if defined(Windows)
# if defined(MOS_TRACK_ALLOCATION_DETAILS)
		ptr(arg, "%s:%d (%s)\n", ma->ma_file, ma->ma_line, ma->ma_func);
#  if defined(TRACK_BACKTRACE)
		print_backtrace(ptr, arg, ma);
#  endif
# endif /* MOS_TRACK_ALLOCATION_DETAILS */
#else /* Windows */
# if defined(MOS_TRACK_ALLOCATION_DETAILS)
		if (ma->ma_backtracesz <= 0)
			ptr(arg, "%s:%d (%s)", ma->ma_file, ma->ma_line, ma->ma_func);
#  if defined(TRACK_BACKTRACE)
		else
			print_backtrace(ptr, arg, ma);
#  endif
# endif
#endif /* Windows */
		ptr(arg, "\n");
		sz += ma->ma_sz;
		acnt++;
	}
	ptr(arg, "total allocations: %u (%u)\n", (unsigned)acnt, (unsigned)sz);
	mos_mutex_unlock(lk);
}

MOSAPI void MOSCConv
mos_dump_allocation_set(uint32_t set, mos_malloc_printer_t ptr, void *arg) {

	ptr(arg, "\nMETA Allocations: %u\n", (unsigned)metaallocs);

	dump_allocation(&tree128, &tree128lk, ptr, arg, set, "128 byte allocations");
	dump_allocation(&tree1024, &tree1024lk, ptr, arg, set, "1024 byte allocations");
	dump_allocation(&treeBIG, &treeBIGlk, ptr, arg, set, "BIG allocations");
}

static int
localprint(void *arg, const char *fmt, ...) {
	va_list va;
	int res;

	va_start(va, fmt);
	res = mos_raw_vprintf(fmt, va);
	va_end(va);

	return (res);
}

MOSAPI void MOSCConv
mos_dump_allocations() {

	mos_dump_allocation_set(0, localprint, NULL);
}

MOSAPI size_t MOSCConv
mos_allocated_bytes(void) {
	struct mos_allocation *ma;
	size_t sz;

	sz = 0;

	mos_mutex_lock(&tree128lk);
	RB_FOREACH(ma, mos_allocation_tree, &tree128)
		sz += ma->ma_sz;
	mos_mutex_unlock(&tree128lk);
	mos_mutex_lock(&tree1024lk);
	RB_FOREACH(ma, mos_allocation_tree, &tree1024)
		sz += ma->ma_sz;
	mos_mutex_unlock(&tree1024lk);
	mos_mutex_lock(&treeBIGlk);
	RB_FOREACH(ma, mos_allocation_tree, &treeBIG)
		sz += ma->ma_sz;
	mos_mutex_unlock(&treeBIGlk);

	return (sz);
}

MOSAPI void MOSCConv
mos_mallocset_set(uint32_t s) {

	mos_mutex_lock(&freelistlk);
	malloc_set = s;
	mos_mutex_unlock(&freelistlk);
}

MOSAPI uint32_t MOSCConv
mos_mallocset_get() {
	uint32_t ret;

	mos_mutex_lock(&freelistlk);
	ret = malloc_set;
	mos_mutex_unlock(&freelistlk);

	return (ret);
}

/*
 * freelistlk must be held during this call.
 */
static void
check_allocations() {
	struct mos_allocation *ma;

	while (freelistsz > MOS_MAX_FREELISTSZ) {
		ma = MTAILQ_FIRST(&freelist);
		MOS_ASSERT(ma != NULL);
		MTAILQ_REMOVE(&freelist, ma, ma_llink);
		mos__free(ma, sizeof (*ma));
		metaallocs -= sizeof (*ma);
		freelistsz--;
	}
}

static struct mos_allocation *
new_allocation() {
	struct mos_allocation *ma;

	ma = mos__alloc(sizeof (*ma), MOSM_SAFE | MOSM_TAG_ALLOCINFO);

	/* pass allocation failure up */
	if (ma == NULL)
		return (NULL);

	metaallocs += sizeof (*ma);

	return (ma);
}

static struct mos_allocation *
get_allocation(size_t sz, void *addr, int flags, const char *fl,
  const char *fn, int ln) {
	struct mos_allocation *ma;

	mos_mutex_lock(&freelistlk);
	ma = MTAILQ_FIRST(&freelist);
	if (ma == NULL) {
		ma = new_allocation(); /* protected by freelistlk */
		mos_mutex_unlock(&freelistlk);
		/* pass allocation failure up */
		if (ma == NULL)
			return (NULL);
		goto setup_alloc;
	}

	/* everything on the freelist should have this size */
	MOS_ASSERT(ma->ma_sz == 0x7fffffff);

	MTAILQ_REMOVE(&freelist, ma, ma_llink);
	MOS_ASSERT(freelistsz > 0);
	freelistsz--;

	mos_mutex_unlock(&freelistlk);

setup_alloc:

	ma->ma_sz = sz;
	ma->ma_ptr = addr;
	ma->ma_set = malloc_set;

#if defined(MOS_TRACK_ALLOCATION_DETAILS)
	ma->ma_flags = flags;
	ma->ma_file = fl;
	ma->ma_func = fn;
	ma->ma_line = ln;
	get_backtrace(ma);
#endif

	return (ma);
}

#define ADDALLOC(lk, tr, ma, sz)	do {								\
	struct mos_allocation *oldma;										\
																		\
	mos_mutex_lock((lk));												\
	oldma = RB_INSERT(mos_allocation_tree, (tr), (ma));					\
	if (oldma != NULL)													\
		MOS_PANIC("attempt to insert allocation at duplicate address");	\
	(sz)++;																\
	mos_mutex_unlock((lk));												\
} while (0)

static void *
register_allocation(size_t sz, void *addr, int flags, const char *fl,
  const char *fn, int ln) {
	struct mos_allocation *ma;

	if (addr == NULL)
		return (NULL);

	ma = get_allocation(sz, addr, flags, fl, fn, ln);

	/* pass allocation failure up */
	if (ma == NULL)
		return (NULL);

	if (sz <= 128)
		ADDALLOC(&tree128lk, &tree128, ma, tree128sz);
	else if (sz <= 1024)
		ADDALLOC(&tree1024lk, &tree1024, ma, tree1024sz);
	else
		ADDALLOC(&treeBIGlk, &treeBIG, ma, treeBIGsz);

	return (addr);
}

#define CKALLOC(l, t, tz, m, s, a, k)	do {		\
	mos_mutex_lock((l));							\
	(m) = RB_FIND(mos_allocation_tree, (t), (k));	\
	MOS_ASSERT((m) != NULL);						\
	MOS_ASSERT((m)->ma_ptr == (a));					\
	MOS_ASSERT((m)->ma_sz == (s));					\
	RB_REMOVE(mos_allocation_tree, (t), m);			\
	(tz)--;											\
	MOS_ASSERT((tz) >= 0);							\
	mos_mutex_unlock((l));							\
} while (0)

static void
deregister_allocation(size_t sz, void *addr, const char *fl, const char *fn, int ln) {
	struct mos_allocation *ma;
	struct mos_allocation key;

	key.ma_ptr = addr;

	mos_mutex_lock(&freelistlk);

	if (sz <= 128)
		CKALLOC(&tree128lk, &tree128, tree128sz, ma, sz, addr, &key);
	else if (sz <= 1024)
		CKALLOC(&tree1024lk, &tree1024, tree1024sz, ma, sz, addr, &key);
	else
		CKALLOC(&treeBIGlk, &treeBIG, treeBIGsz, ma, sz, addr, &key);

	ma->ma_sz = 0x7fffffff;

	MTAILQ_INSERT_HEAD(&freelist, ma, ma_llink);
	freelistsz++;
	check_allocations();

	mos_mutex_unlock(&freelistlk);
}


void
_mos_malloc_init() {

	_mos_malloc_os_init();
	MTAILQ_INIT(&freelist);
	RB_INIT(&tree128);
	RB_INIT(&tree1024);
	RB_INIT(&treeBIG);

	mos_mutex_init(&freelistlk);
	mos_mutex_init(&tree128lk);
	mos_mutex_init(&tree1024lk);
	mos_mutex_init(&treeBIGlk);
}

void
_mos_malloc_fini() {
	struct mos_allocation *ma;

	mos_mutex_destroy(&freelistlk);
	mos_mutex_destroy(&tree128lk);
	mos_mutex_destroy(&tree1024lk);
	mos_mutex_destroy(&treeBIGlk);

	while (freelistsz > 0) {
		ma = MTAILQ_FIRST(&freelist);
		MOS_ASSERT(ma != NULL);
		MTAILQ_REMOVE(&freelist, ma, ma_llink);
		mos__free(ma, sizeof (*ma));
		metaallocs -= sizeof (*ma);
		freelistsz--;
	}

	MOS_ASSERT(MTAILQ_EMPTY(&freelist));
	_mos_malloc_os_fini();
}

MOSAPI void * MOSCConv
_mos_alloc(size_t sz, int flags, const char *file, const char *func, int line) {
	void *ptr;

	if (flags == 0)
		MOS_PANIC("alloc flags are 0");

	if (flags & MOSM_SLP && flags & MOSM_NSLP)
		MOS_PANIC("sleep and nosleep alloc flags set");

	if (flags & MOSM_PG && flags & MOSM_NPG)
		MOS_PANIC("page and nonpage alloc flags set");

again:

	ptr = mos__alloc(sz, flags);
	if (ptr == NULL)
		return (NULL);

	if (register_allocation(sz, ptr, flags, file, func, line) == NULL) {
		/* allocation failure; retry if possible or return */
		mos__free(ptr, sz);
		if (flags & MOSM_SLP)
			goto again;
	}

	return (ptr);
}

MOSAPI void MOSCConv
_mos_free(void *ptr, size_t sz, const char *file, const char *func, int line) {

	MOS_ASSERT(ptr != NULL);

	if (sz == MOSM_FSTR)
		sz = mos_strlen((const char *)ptr) + 1;

	deregister_allocation(sz, ptr, file, func, line);
	mos__free(ptr, sz);
}

MOSAPI size_t MOSCConv
mos_print_allocated_bytes(void) {
	size_t sz;

	sz = mos_allocated_bytes();

	if (sz > 0)
		mos_raw_printf("MEMORY DEBUGGING: %llu allocated bytes unfreed\n", (ull_t)sz);

	return (sz);
}

#else /* MOS_TRACK_ALLOCATIONS */

/*
 * Basic implementations, that do not deal with tracking allocations.
 */

void
_mos_malloc_init() {

	_mos_malloc_os_init();
}

void
_mos_malloc_fini() {

	_mos_malloc_os_fini();
}

MOSAPI void * MOSCConv
_mos_alloc(size_t sz, int flags, const char *file, const char *func, int line) {

	if (flags == 0)
		MOS_PANIC("alloc flags are 0");

	if (flags & MOSM_SLP && flags & MOSM_NSLP)
		MOS_PANIC("sleep and nosleep alloc flags set");

	if (flags & MOSM_PG && flags & MOSM_NPG)
		MOS_PANIC("page and nonpage alloc flags set");

	return (mos__alloc(sz, flags));
}

MOSAPI void MOSCConv
_mos_free(void *ptr, size_t sz, const char *file, const char *func, int line) {

	mos__free(ptr, sz);
}

MOSAPI void MOSCConv
mos_dump_allocation_set(uint32_t set, mos_malloc_printer_t ptr, void *arg) {
}

MOSAPI void MOSCConv
mos_dump_allocations() {
}

MOSAPI size_t MOSCConv
mos_allocated_bytes(void) {

	return (0);
}

MOSAPI void MOSCConv
mos_mallocset_set(uint32_t s) {
}

MOSAPI uint32_t MOSCConv
mos_mallocset_get() {

	return (0);
}

MOSAPI size_t MOSCConv
mos_print_allocated_bytes(void) {

	return (0);
}

#endif /* MOS_TRACK_ALLOCATIONS */

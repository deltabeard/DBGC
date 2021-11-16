#pragma once

/* Helpers. */
/* Deliberately unused return value. */
#define UNUSED_RET	(void)
/* Number of elements within an array. */
#define ARRAYSIZE(x)	(sizeof(x)/sizeof(x[0]))

/* Optimisations. */
/* Branch is likely to happen. */
#define LIKELY(expr)	__builtin_expect(!!(expr), 1)
/* Branch is unlikely to happen. */
#define UNLIKELY(expr)	__builtin_expect(!!(expr), 0)
/* Reaching this part of the code is impossible and undefined. */
#define UNREACHABLE()	__builtin_unreachable()
/* Inline a function. May be ignored by the compiler. */
#define INLINE		__inline__
/* Forcibly inline a function. Useful for inlining into time-critical
 * functions. */
#define ALWAYS_INLINE	__attribute__((__always_inline__)) INLINE

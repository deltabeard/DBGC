#pragma once

/* Helpers. */
#define UNUSED(x)	(void)(x)
#define ARRAYSIZE(x)	(sizeof(x)/sizeof(x[0]))

/* Optimisations. */
#define LIKELY(expr)	__builtin_expect(!!(expr), 1)
#define UNLIKELY(expr)	__builtin_expect(!!(expr), 0)
#define UNREACHABLE()	__builtin_unreachable()
#define INLINE		__inline__
#define ALWAYS_INLINE	__attribute__((__always_inline__)) INLINE

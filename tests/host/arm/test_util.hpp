#pragma once
// 実機なしでロジックを検証するための最小の土台。
//
// 失敗しても止めない。1 回の実行でどこがどう食い違ったかを全部見せてから、
// report() が 0 以外を返す。期待値と実測値は必ず両方印字する。
// 「通った/落ちた」だけ出すテストは、落ちたときに何の役にも立たない。

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstddef>
#include <initializer_list>

namespace test {

namespace detail {
inline int checks   = 0;
inline int failures = 0;

inline void pass() { ++checks; }

inline void fail(const char* what) {
    ++checks;
    ++failures;
    std::printf("  NG  %s\n", what);
}

inline void dumpBytes(const char* label, const uint8_t* p, std::size_t n) {
    std::printf("      %-8s", label);
    for (std::size_t i = 0; i < n; ++i) std::printf(" %02X", p[i]);
    std::printf("\n");
}
}  // namespace detail

// 節の見出し。**その場で吐き出す。** 落ちたときにどこまで進んだかが分からないと、
// 原因の切り分けにデバッガが要る。
inline void section(const char* name) {
    std::printf("--- %s\n", name);
    std::fflush(stdout);
}

inline bool check(bool ok, const char* what) {
    if (ok) {
        detail::pass();
        return true;
    }
    detail::fail(what);
    return false;
}

// 符号なし整数。食い違ったら 10 進と 16 進の両方を出す。
inline bool checkEq(uint64_t got, uint64_t want, const char* what) {
    if (got == want) {
        detail::pass();
        return true;
    }
    detail::fail(what);
    std::printf("      期待 %llu (0x%llX) / 実際 %llu (0x%llX)\n",
                static_cast<unsigned long long>(want), static_cast<unsigned long long>(want),
                static_cast<unsigned long long>(got), static_cast<unsigned long long>(got));
    return false;
}

// 符号つき整数。負値を uint64_t に落として比較する事故を防ぐため別関数にする。
inline bool checkEqI(int64_t got, int64_t want, const char* what) {
    if (got == want) {
        detail::pass();
        return true;
    }
    detail::fail(what);
    std::printf("      期待 %lld / 実際 %lld\n", static_cast<long long>(want),
                static_cast<long long>(got));
    return false;
}

inline bool checkNear(double got, double want, double tol, const char* what) {
    if (std::isfinite(got) && std::fabs(got - want) <= tol) {
        detail::pass();
        return true;
    }
    detail::fail(what);
    std::printf("      期待 %.6f ± %.6f / 実際 %.6f\n", want, tol, got);
    return false;
}

// 公式マニュアルの実例フレームと直接照合するための比較。
// 差分が一目で分かるように両方を 16 進で並べる。
inline bool checkBytes(const uint8_t* got, const uint8_t* want, std::size_t n, const char* what) {
    if (std::memcmp(got, want, n) == 0) {
        detail::pass();
        return true;
    }
    detail::fail(what);
    detail::dumpBytes("期待", want, n);
    detail::dumpBytes("実際", got, n);
    return false;
}

inline bool checkBytes(const uint8_t* got, std::initializer_list<uint8_t> want, const char* what) {
    uint8_t buf[16] = {};
    std::size_t n   = want.size() < sizeof(buf) ? want.size() : sizeof(buf);
    std::size_t i   = 0;
    for (uint8_t b : want) {
        if (i >= n) break;
        buf[i++] = b;
    }
    return checkBytes(got, buf, n, what);
}

// float のまま比較する。
// -Wdouble-promotion を有効にしたテストでは、checkNear(double,...) を呼ぶだけで
// 呼び出し側に警告が出る。ここで受けて中で 1 度だけ広げる。
inline bool checkNearF(float got, float want, float tol, const char* what) {
    if (fabsf(got - want) <= tol) {
        detail::pass();
        return true;
    }
    detail::fail(what);
    std::printf("      期待 %.6f / 実際 %.6f (許容 %.6f)\n", static_cast<double>(want),
                static_cast<double>(got), static_cast<double>(tol));
    return false;
}

inline bool checkStr(const char* got, const char* want, const char* what) {
    if (got != nullptr && want != nullptr && std::strcmp(got, want) == 0) {
        detail::pass();
        return true;
    }
    detail::fail(what);
    std::printf("      期待 \"%s\" / 実際 \"%s\"\n", want ? want : "(null)", got ? got : "(null)");
    return false;
}

inline int report(const char* suite) {
    std::printf("\n%s: %d checks, %d failures\n", suite, detail::checks, detail::failures);
    return detail::failures == 0 ? 0 : 1;
}

}  // namespace test


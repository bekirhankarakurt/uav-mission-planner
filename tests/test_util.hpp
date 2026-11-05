#pragma once

// Tiny dependency-free unit test harness.

#include <cmath>
#include <exception>
#include <iostream>
#include <vector>

namespace ut {

struct Case {
  const char* name;
  void (*fn)();
};

inline std::vector<Case>& registry() {
  static std::vector<Case> cases;
  return cases;
}

inline int& failures() {
  static int count = 0;
  return count;
}

struct Registrar {
  Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

inline bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

inline int runAll() {
  for (const Case& c : registry()) {
    const int before = failures();
    try {
      c.fn();
    } catch (const std::exception& e) {
      ++failures();
      std::cerr << "  unexpected exception: " << e.what() << "\n";
    }
    std::cout << (failures() == before ? "[ OK ] " : "[FAIL] ") << c.name << "\n";
  }
  std::cout << registry().size() << " tests, " << failures() << " failed checks\n";
  return failures() == 0 ? 0 : 1;
}

}  // namespace ut

#define TEST(name)                                       \
  static void name();                                    \
  static ut::Registrar registrar_##name(#name, &name);   \
  static void name()

#define CHECK(cond)                                                                 \
  do {                                                                              \
    if (!(cond)) {                                                                  \
      ++ut::failures();                                                             \
      std::cerr << __FILE__ << ":" << __LINE__ << ": CHECK failed: " #cond "\n";    \
    }                                                                               \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                    \
  do {                                                                                           \
    const double va_ = (a);                                                                      \
    const double vb_ = (b);                                                                      \
    if (!ut::near(va_, vb_, (tol))) {                                                            \
      ++ut::failures();                                                                          \
      std::cerr << __FILE__ << ":" << __LINE__ << ": CHECK_NEAR failed: " #a " = " << va_       \
                << ", " #b " = " << vb_ << "\n";                                                 \
    }                                                                                            \
  } while (0)

#define CHECK_THROWS(expr)                  \
  do {                                      \
    bool thrown_ = false;                   \
    try {                                   \
      (void)(expr);                         \
    } catch (const std::exception&) {       \
      thrown_ = true;                       \
    }                                       \
    CHECK(thrown_);                         \
  } while (0)

#define TEST_MAIN() \
  int main() { return ut::runAll(); }

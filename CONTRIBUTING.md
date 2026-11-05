# Contributing

Thanks for your interest in improving uav-mission-planner. Bug reports, feature ideas and pull
requests are welcome.

## Building and testing

C++ core, CLI and unit tests (CMake 3.16+, a C++17 compiler, no other dependencies):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUAV_WARNINGS_AS_ERRORS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Sanitizers:

```bash
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DUAV_SANITIZE=ON
cmake --build build-asan --parallel && ctest --test-dir build-asan --output-on-failure
```

Python package (builds the pybind11 extension; needs network access for the pinned pybind11 tag):

```bash
pip install ".[test,plot]"
python -m pytest
```

## Guidelines

- The code is warning-free with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`; keep it that way.
- Use RAII and standard containers; no raw `new` / `delete`. Prefer `enum class` and mark
  value-returning functions `[[nodiscard]]`.
- The core library has no third-party dependencies. pybind11 is fetched only for the Python module.
- Every behavioural change needs a test: C++ tests live in `tests/`, Python tests in `python/tests/`.
- Keep commits small and focused, with an imperative subject line (for example "Fix void filling at tile edges").
- Update `CHANGELOG.md` for user-visible changes.

## Pull requests

1. Fork the repository and create a topic branch.
2. Make sure the C++ tests and the Python tests pass locally.
3. Open a pull request describing the problem and the approach. CI must be green before merging.

By contributing you agree that your contributions are licensed under the MIT License.

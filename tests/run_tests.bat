@echo off
REM Syringe Pump Unit Test Runner
echo === Configuring Unit Tests ===
cmake -B build/tests -S tests -G Ninja -DCMAKE_C_COMPILER=gcc
if errorlevel 1 goto error

echo.
echo === Building Unit Tests ===
ninja -C build/tests
if errorlevel 1 goto error

echo.
echo === Running Tests via CTest ===
ctest --test-dir build/tests --output-on-failure
if errorlevel 1 goto error

echo.
echo [SUCCESS] All unit tests passed!
exit /b 0

:error
echo.
echo [FAILED] Unit tests build or execution failed.
exit /b 1

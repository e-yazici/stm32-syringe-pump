# Syringe Pump Unit Test Runner (PowerShell)
Write-Host "=== Configuring Unit Tests ===" -ForegroundColor Cyan
cmake -B build/tests -S tests -G Ninja -DCMAKE_C_COMPILER=gcc
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`n=== Building Unit Tests ===" -ForegroundColor Cyan
ninja -C build/tests
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`n=== Running Tests via CTest ===" -ForegroundColor Cyan
ctest --test-dir build/tests --output-on-failure
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host "`n[SUCCESS] All unit tests passed!" -ForegroundColor Green

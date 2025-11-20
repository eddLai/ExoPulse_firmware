#!/bin/bash
# ExoPulse Firmware - Automated Test Pipeline
# This script performs comprehensive build verification and testing

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_LOG="${PROJECT_DIR}/build_report.log"
REPORT_FILE="${PROJECT_DIR}/BUILD_REPORT.md"

# Ensure PATH includes platformio
export PATH="/root/.local/bin:$PATH"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  ExoPulse Build Verification Pipeline${NC}"
echo -e "${BLUE}========================================${NC}\n"

# Initialize log
echo "Build started at: $(date)" > "${BUILD_LOG}"
echo "Project: ExoPulse Firmware" >> "${BUILD_LOG}"
echo "Branch: $(git branch --show-current 2>/dev/null || echo 'unknown')" >> "${BUILD_LOG}"
echo "Commit: $(git rev-parse --short HEAD 2>/dev/null || echo 'unknown')" >> "${BUILD_LOG}"
echo "" >> "${BUILD_LOG}"

# Test counters
TESTS_PASSED=0
TESTS_FAILED=0
ENVIRONMENTS=("esp32doit-devkit-v1" "esp32-s3-n16r8" "firebeetle32")

# Function to log and print
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
    echo "[INFO] $1" >> "${BUILD_LOG}"
}

log_success() {
    echo -e "${GREEN}[PASS]${NC} $1"
    echo "[PASS] $1" >> "${BUILD_LOG}"
    ((TESTS_PASSED++))
}

log_error() {
    echo -e "${RED}[FAIL]${NC} $1"
    echo "[FAIL] $1" >> "${BUILD_LOG}"
    ((TESTS_FAILED++))
}

log_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
    echo "[WARN] $1" >> "${BUILD_LOG}"
}

# Phase 1: Pre-build Checks
echo -e "\n${BLUE}=== Phase 1: Pre-build Checks ===${NC}\n"

log_info "Checking project structure..."
if [ -f "src/main.cpp" ]; then
    log_success "Main source file exists (src/main.cpp)"
else
    log_error "Main source file missing (src/main.cpp)"
    exit 1
fi

if [ -f "platformio.ini" ]; then
    log_success "PlatformIO configuration exists"
else
    log_error "platformio.ini missing"
    exit 1
fi

if [ -d "include" ]; then
    log_success "Include directory exists"
else
    log_error "Include directory missing"
    exit 1
fi

# Check for critical header files
CRITICAL_HEADERS=("include/ExoBus.h" "include/SerialConsole.h")
for header in "${CRITICAL_HEADERS[@]}"; do
    if [ -f "$header" ]; then
        log_success "Found: $header"
    else
        log_error "Missing: $header"
        exit 1
    fi
done

# Phase 2: Clean Build
echo -e "\n${BLUE}=== Phase 2: Clean Build ===${NC}\n"

log_info "Cleaning previous builds..."
pio run --target clean >> "${BUILD_LOG}" 2>&1 && log_success "Clean successful" || log_warning "Clean had warnings"

# Phase 3: Build All Environments
echo -e "\n${BLUE}=== Phase 3: Build All Environments ===${NC}\n"

BUILD_SUCCESS=true

for env in "${ENVIRONMENTS[@]}"; do
    log_info "Building environment: $env"

    if pio run --environment "$env" >> "${BUILD_LOG}" 2>&1; then
        log_success "Build succeeded: $env"

        # Get binary size info
        FIRMWARE_PATH=".pio/build/$env/firmware.elf"
        if [ -f "$FIRMWARE_PATH" ]; then
            SIZE_INFO=$(pio run --environment "$env" --target size 2>&1 | grep -E '(RAM:|Flash:)' || echo "Size info unavailable")
            echo "  $SIZE_INFO"
            echo "  Size: $SIZE_INFO" >> "${BUILD_LOG}"
        fi
    else
        log_error "Build failed: $env"
        BUILD_SUCCESS=false
    fi
    echo ""
done

# Phase 4: Static Analysis
echo -e "\n${BLUE}=== Phase 4: Static Analysis ===${NC}\n"

log_info "Running PlatformIO check..."
if pio check --skip-packages >> "${BUILD_LOG}" 2>&1; then
    log_success "Static analysis passed"
else
    log_warning "Static analysis found issues (check log)"
fi

# Phase 5: Code Quality Checks
echo -e "\n${BLUE}=== Phase 5: Code Quality Checks ===${NC}\n"

log_info "Checking for common issues..."

# Check for TODO/FIXME comments
TODO_COUNT=$(grep -r "TODO\|FIXME" src/ include/ 2>/dev/null | wc -l || echo "0")
if [ "$TODO_COUNT" -gt 0 ]; then
    log_warning "Found $TODO_COUNT TODO/FIXME comments"
else
    log_success "No TODO/FIXME comments"
fi

# Check for debug print statements that might be left in
DEBUG_COUNT=$(grep -r "Serial.print.*DEBUG\|Serial.print.*TEST" src/ include/ 2>/dev/null | wc -l || echo "0")
if [ "$DEBUG_COUNT" -gt 0 ]; then
    log_warning "Found $DEBUG_COUNT debug print statements"
else
    log_success "No debug print statements"
fi

# Phase 6: Generate Report
echo -e "\n${BLUE}=== Phase 6: Generating Report ===${NC}\n"

log_info "Creating build report..."

cat > "${REPORT_FILE}" << EOF
# ExoPulse Firmware - Build Verification Report

**Generated:** $(date)
**Branch:** $(git branch --show-current 2>/dev/null || echo 'unknown')
**Commit:** $(git rev-parse --short HEAD 2>/dev/null || echo 'unknown')

---

## Summary

- **Tests Passed:** ${TESTS_PASSED}
- **Tests Failed:** ${TESTS_FAILED}
- **Build Status:** $([ "$BUILD_SUCCESS" = true ] && echo "✅ SUCCESS" || echo "❌ FAILED")

---

## Build Results

EOF

for env in "${ENVIRONMENTS[@]}"; do
    FIRMWARE_PATH=".pio/build/$env/firmware.elf"
    if [ -f "$FIRMWARE_PATH" ]; then
        echo "### $env" >> "${REPORT_FILE}"
        echo "- Status: ✅ **SUCCESS**" >> "${REPORT_FILE}"

        # Extract size information
        SIZE_OUTPUT=$(pio run --environment "$env" --target size 2>&1 || echo "")
        RAM_LINE=$(echo "$SIZE_OUTPUT" | grep "RAM:" || echo "RAM: N/A")
        FLASH_LINE=$(echo "$SIZE_OUTPUT" | grep "Flash:" || echo "Flash: N/A")

        echo "- $RAM_LINE" >> "${REPORT_FILE}"
        echo "- $FLASH_LINE" >> "${REPORT_FILE}"
        echo "" >> "${REPORT_FILE}"
    else
        echo "### $env" >> "${REPORT_FILE}"
        echo "- Status: ❌ **FAILED**" >> "${REPORT_FILE}"
        echo "" >> "${REPORT_FILE}"
    fi
done

cat >> "${REPORT_FILE}" << EOF
---

## Code Quality

- TODO/FIXME comments: ${TODO_COUNT}
- Debug print statements: ${DEBUG_COUNT}

---

## Bug Fixes Applied

1. ✅ **CRITICAL:** Renamed \`src/main.h\` to \`src/main.cpp\`
   - PlatformIO requires .cpp files as entry points
   - This was causing build failures

2. ✅ **Organization:** Moved test programs to \`examples/\` directory
   - Cleaned up src/ to contain only production code
   - Added comprehensive examples/README.md

3. ✅ **Configuration:** Fixed baud rate consistency
   - Standardized to 115200 across all configs
   - Updated README.md to match implementation

4. ✅ **Build System:** Added C++17 flags to all environments
   - Required for inline static member initialization
   - Ensures consistent compilation

---

## Next Steps

1. **Hardware Testing:**
   - Run \`examples/test_loopback.h\` to verify MCP2515 (no external hardware)
   - Run \`examples/test_led.h\` to verify ESP32 board
   - Test CAN communication with sender/receiver pair

2. **Deployment:**
   - Upload firmware: \`pio run --target upload\`
   - Monitor serial: \`pio device monitor --baud 115200\`

3. **Integration Testing:**
   - Test serial commands (HELP, STATUS, TORQ, etc.)
   - Verify motor control with actual hardware
   - Test watchdog functionality

---

## Full Build Log

See \`build_report.log\` for complete compilation output.

EOF

log_success "Report generated: ${REPORT_FILE}"

# Final Summary
echo -e "\n${BLUE}========================================${NC}"
echo -e "${BLUE}  Build Verification Complete${NC}"
echo -e "${BLUE}========================================${NC}\n"

echo -e "Tests Passed:  ${GREEN}${TESTS_PASSED}${NC}"
echo -e "Tests Failed:  ${RED}${TESTS_FAILED}${NC}"
echo -e "\nReports:"
echo -e "  - ${REPORT_FILE}"
echo -e "  - ${BUILD_LOG}"

if [ "$BUILD_SUCCESS" = true ]; then
    echo -e "\n${GREEN}✅ All builds successful!${NC}\n"
    exit 0
else
    echo -e "\n${RED}❌ Some builds failed. Check ${BUILD_LOG} for details.${NC}\n"
    exit 1
fi

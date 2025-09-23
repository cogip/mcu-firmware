# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
This is the MCU firmware for Cogip robot used in Eurobot competition. It runs on STM32F4xx microcontrollers and is built on top of RIOT-OS.

## Build System Setup

### Prerequisites
1. RIOT-OS repository must be cloned alongside this repository:
   ```bash
   git clone https://github.com/cogip/RIOT.git -b cogip_master ../RIOT
   ```

2. Apply RIOT-OS patches before any build:
   ```bash
   make riot-patches
   ```

3. Python virtual environment setup:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

## Common Build Commands

### Building Applications
```bash
# Build specific application with default board (cogip-board)
make -j$(nproc) -C applications/<app_name>

# Build with specific board
make -j$(nproc) BOARD=<board_name> -C applications/<app_name>

# Available boards: cogip-board, cogip-native
# Example: make -j$(nproc) BOARD=cogip-native -C applications/robot-motion-control
```

### Building with Robot ID
```bash
# Applications require ROBOT_ID parameter (defaults to 1)
make -j$(nproc) ROBOT_ID=2 -C applications/robot-motion-control
```

### Building Everything
```bash
# Build all applications and examples
make

# Clean build
make clean

# Deep clean (removes all build artifacts)
make distclean
```

### Flash to Board
```bash
make -j$(nproc) BOARD=cogip-board -C applications/<app_name> flash
```

### Debug (Native Board)
```bash
make -j$(nproc) BOARD=cogip-native -C applications/<app_name> all-debug
```

## Code Quality

### Check Coding Rules
```bash
make check-codingrules
```

### Apply Coding Rules
```bash
tools/check-codingrules.sh apply
```

## Architecture

### Directory Structure
- **applications/**: Main robot applications
  - cup2024-*: Competition-specific applications for 2024
  - cup2025-*: Competition-specific applications for 2025
  - robot-motion-control: Main motion control application
  - power-supply-control: Power management application

- **boards/**: Board-specific configurations
  - cogip-board: Physical STM32F4 board
  - cogip-native: Native simulation board for x86_64

- **motion_control/**: Motion control subsystem
  - controllers/: PID and other control algorithms
  - engines/: Motion engines implementation
  - filters/: Signal filtering implementations
  - metas/: Meta-controller implementations
  - motion_control_common/: Shared motion control components

- **platforms/**: Platform-specific configurations combining hardware and software modules
  - pf-robot-motion-control: Platform for motion control robots
  - pf-robot-motors: Platform for motor control
  - pf-power-supply: Platform for power supply management

- **lib/**: Shared libraries and utilities
- **sys/**: System modules (CAN protocol buffer, sysmon, threading)
- **pkg/**: External packages
- **examples/**: Example applications for testing specific features

### Key Build Variables
- `BOARD`: Target board (cogip-board or cogip-native)
- `ROBOT_ID`: Robot identifier for multi-robot setups
- `RIOTBASE`: Path to RIOT-OS repository (defaults to ../RIOT)
- `MCUFIRMWAREBASE`: Path to this firmware repository

### Module System
The project uses RIOT-OS module system. Modules are loaded via:
- `USEMODULE` in Makefiles
- Platform modules in platforms/pf-*/
- External modules defined in `EXTERNAL_MODULE_DIRS`

## Testing
Run example applications in examples/ directory for testing specific features:
```bash
make -j$(nproc) -C examples/motion_control_robot_test
```

## Important Notes
- Always ensure RIOT patches are applied before building
- The project uses C++17 standard
- Default log level is set to LOG_ERROR
- Main thread stack size is 8192 bytes
- ISR stack size is 2048 bytes (increased for libstdc++)
- **ETL Usage**: The project uses ETL (Embedded Template Library) when possible instead of STL:
  - Use `etl::absolute` instead of `std::fabs/std::abs`
  - Use `etl::min/max` instead of `std::min/max`
  - Use `etl::numeric_limits` instead of `std::numeric_limits`
  - Use `etl::any_of` instead of `std::any_of`
  - Standard streams (`std::cout/cerr/endl`) and math functions (`std::sqrt`) remain as std::
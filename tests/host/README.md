# Fixed PID host checks

From the project root, using a native MinGW g++ (not arm-none-eabi-g++):

```powershell
New-Item -ItemType Directory -Force build/host-tests | Out-Null
g++ -std=c++17 -Wall -Wextra -Werror -static -I tests/host/stubs -I User/PID -I User/CAN -I User/Task tests/host/fixed_pid_test.cpp User/PID/PID.cpp User/PID/Debug.cpp User/Task/Gimbal.cpp -o build/host-tests/fixed_pid_test.exe
if ($LASTEXITCODE -eq 0) { & ./build/host-tests/fixed_pid_test.exe }
```

Uses the real PID, axis, UART command parser and control task with fake HAL/FreeRTOS
functions. Checks boot STOP, accepted/rejected commands, switching between manual
current and position control, resetting PID state, feedback loss, cancelling an old
command when a stop arrives, shortest-angle wrapping, fractional target speed and
the position mode current limit.

These checks do not model the motor, actual CAN delivery, UART fragmentation,
interrupt timing, feedback delay or physical closed-loop stability. The STM32
firmware build remains a separate check. These files are not in the firmware target.

The axis assertions reflect the latest user-verified gains (position Kp=0.1,
Ki=0.005; speed Kp=5, Ki=0.002) and the 8000 current command limit. If those settings
are intentionally changed, update the corresponding expected values.

## RBF observer checks

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -static -I User/PID tests/host/rbf_test.cpp -o build/host-tests/rbf_test.exe
if ($LASTEXITCODE -eq 0) { & ./build/host-tests/rbf_test.exe }
```

Checks learning on a synthetic response using separate training/validation input
sets, rejection of non-finite/out-of-range values, scoring before learning,
freeze/off/reset, repeated CAN samples, timing gaps and timer wrap-around.
The fixed PID test also compares the real task's outputs with NN off versus learning
and verifies that failed CAN queue requests do not become training samples.

Synthetic accuracy says nothing about GM6020 prediction quality. Motor logs after
`nn=freeze` are needed for a held-out hardware comparison against the constant-speed
baseline. No gain update or autotuning is performed in this stage.

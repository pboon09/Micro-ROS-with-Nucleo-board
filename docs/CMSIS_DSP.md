# CMSIS-DSP

[CMSIS-DSP](https://github.com/ARM-software/CMSIS-DSP) is ARM's math library for Cortex-M: filters, matrices, transforms and statistics. It uses the STM32G474's hardware FPU.

## In a project

It is already there, in [`Middlewares/Third_Party/ARM_CMSIS/`](../uros_example/Middlewares/Third_Party/ARM_CMSIS), with the include paths set. Just use it.

```c
#include "arm_math.h"

float32_t in[4] = {1.0f, 2.0f, 3.0f, 4.0f};
float32_t mean;
arm_mean_f32(in, 4, &mean);  // 2.5
```

## In a blank project

The packs are in [`CMSIS/`](../CMSIS). [`cmsis_install.pdf`](../CMSIS/cmsis_install.pdf) has the same steps with screenshots.

1. **Help > Manage Embedded Software Packages > From Local**. Install [`ARM.CMSIS.6.0.0.pack`](../CMSIS/ARM.CMSIS.6.0.0.pack), then [`ARM.CMSIS-DSP.1.16.2.pack`](../CMSIS/ARM.CMSIS-DSP.1.16.2.pack).
2. In the `.ioc`, open **Software Packs**. Tick **CMSIS CORE** under ARM.CMSIS and set ARM.CMSIS-DSP to **Source**. Enable **CMSIS** and **CMSIS-DSP** in the category list, then generate code.
3. Drag `~/STM32Cube/Repository/Packs/ARM/CMSIS-DSP/1.16.2/Source` onto the project root and choose **Copy files and folders**.
4. Right-click `Source` > **Resource Configurations > Exclude from Build**, tick Debug and Release.
5. Right-click `Source` > **Add/remove include path**, select all subfolders, tick Debug and Release.
6. Add `#include "arm_math.h"` and build.

The full function list is in the [CMSIS-DSP docs](https://arm-software.github.io/CMSIS-DSP/latest/).

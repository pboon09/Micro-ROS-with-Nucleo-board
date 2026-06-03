# Adding CMSIS-DSP

[CMSIS-DSP](https://github.com/ARM-software/CMSIS-DSP) is ARM's optimized math
library for Cortex-M (filters, matrices, transforms, statistics, and more). The
STM32G474 is a Cortex-M4F with a single-precision FPU, so it can use the
hardware-accelerated DSP routines.

## What ships in this repo

| Location | What it is |
|---|---|
| [`CMSIS/ARM.CMSIS.6.0.0.pack`](../CMSIS/ARM.CMSIS.6.0.0.pack) | CMSIS core pack, installable in STM32CubeIDE |
| [`CMSIS/ARM.CMSIS-DSP.1.16.2.pack`](../CMSIS/ARM.CMSIS-DSP.1.16.2.pack) | CMSIS-DSP pack, installable in STM32CubeIDE |
| [`CMSIS/cmsis_install.pdf`](../CMSIS/cmsis_install.pdf) | Step-by-step install manual with screenshots |
| `uros_example/Middlewares/Third_Party/ARM_CMSIS/` | The DSP library already vendored into the template (Include, PrivateInclude, Source, Core) |
| `uros_example/Source/` | Upstream CMSIS-DSP source tree with CMake and Makefile build files |

In the template the DSP include paths are already present in the project build
settings, so there is nothing to enable. On a fresh project you install the packs
and add the paths yourself, which is Option B below.

## Option A. Use the version already vendored (template)

There is nothing to install and nothing to enable. The library lives under
`uros_example/Middlewares/Third_Party/ARM_CMSIS/` and its include paths are
already set. Just include the header and call the functions.

```c
#include "arm_math.h"

float32_t in[4]  = {1.0f, 2.0f, 3.0f, 4.0f};
float32_t mean;
arm_mean_f32(in, 4, &mean);   // mean == 2.5f
```

## Option B. Install the packs into STM32CubeIDE (fresh project)

Use this when starting from a blank project. These steps follow
[`cmsis_install.pdf`](../CMSIS/cmsis_install.pdf), which has full screenshots.
The two packs are already in the [`CMSIS/`](../CMSIS) folder of this repo, taken
from the ARM releases of CMSIS 6.0.0 and CMSIS-DSP. Items you click are shown in
**bold**, and files, folders, and code are shown in `code`.

1. **Install each pack.** Open **Help** then **Manage Embedded Software
   Packages**. Click **From Local**, select `CMSIS/ARM.CMSIS.6.0.0.pack`, click
   **Install**, and accept the license. Repeat for
   `CMSIS/ARM.CMSIS-DSP.1.16.2.pack`.

2. **Enable the components in the `.ioc`.** Open the `.ioc`, go to **Pinout and
   Configuration**, then open **Software Packs** and the component selector.
   Under **ARM.CMSIS** tick **CMSIS CORE**. Under **ARM.CMSIS-DSP** set the
   selection to **Source**, then click **Ok**. In the category list on the left,
   enable **CMSIS** and **CMSIS-DSP**.

3. **Regenerate** the project with the gear icon.

4. **Copy the full `Source` folder into the project root.** The generated DSP
   files are incomplete, so copy `Source` from the installed pack. The pack lives
   at one of these locations, where `<version>` is the DSP pack version such as
   `1.16.2`.
   - Windows. `C:\Users\<username>\STM32Cube\Repository\Packs\ARM\CMSIS-DSP\<version>\Source`
   - Linux or macOS. `~/STM32Cube/Repository/Packs/ARM/CMSIS-DSP/<version>/Source`

   Drag that `Source` folder onto the project root and choose **Copy files and
   folders**.

5. **Exclude `Source` from the build.** Right-click the copied `Source` folder,
   choose **Resource Configurations** then **Exclude from Build**, tick **Debug**
   and **Release**, then click **OK**.

6. **Add the include paths.** Right-click `Source` again, choose **Add/remove
   include path**, select all of the function subfolders, tick **Debug** and
   **Release**, then click **OK**.

7. **Include and build.** Add `#include "arm_math.h"` in
   `USER CODE BEGIN Includes` and build. It should compile with no errors.

## What is in the library

The DSP function categories are documented at
[keil.com/pack/doc/CMSIS/DSP](https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html).

- Basic math functions
- Fast math functions
- Complex math functions
- Filtering functions
- Matrix functions
- Transform functions
- Motor control functions
- Statistical functions
- Support functions
- Interpolation functions
- Support Vector Machine functions (SVM)
- Bayes classifier functions
- Distance functions
- Quaternion functions

## Notes

The supported routines vary by CMSIS-DSP version. The authoritative reference is
the [CMSIS-DSP documentation](https://arm-software.github.io/CMSIS-DSP/latest/)
and the included [`cmsis_install.pdf`](../CMSIS/cmsis_install.pdf).

DSP source files can be large to compile. If build time matters, compile only the
function groups you use. The `Source/` subfolders are split by category, such as
`FilteringFunctions`, `MatrixFunctions`, and `StatisticsFunctions`.

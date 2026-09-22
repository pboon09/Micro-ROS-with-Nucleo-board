# Tips and troubleshooting

## Git

The project ships a [`.gitignore`](../uros_example/.gitignore) for `Debug/`, `Release/` and `libmicroros/`. They are rebuilt on every build and `libmicroros/` is tens of MB.

If `Debug/` or `Release/` was committed before, untrack it once. Your files stay on disk.

```bash
git rm -r --cached Debug Release
```

To commit `micro_ros_stm32cubemx_utils` into your own repo, remove its git history first so it does not become a nested repo.

```bash
cd micro_ros_stm32cubemx_utils && rm -rf .git*
```

## Keep colcon out of the firmware

[`create_project.sh`](../create_project.sh) puts the project in `<workspace>/firmware/` with a `COLCON_IGNORE` file, so `colcon build` skips it. For a project you made by hand:

```bash
touch ~/ros2_ws/firmware/COLCON_IGNORE
```

## Troubleshooting

| Problem | Fix |
|---|---|
| Nothing connects | Start the agent first, then press the board's reset button |
| Agent connects but `ros2 topic list` is empty | Check `ROS_DOMAIN_ID` if it matches the agent's and `unset ROS_LOCALHOST_ONLY` |
| Agent never connects | The agent's `-b` must match the LPUART baud rate, 2000000 in the template |
| Service values arrive as garbage, topics are fine | The host uses CycloneDDS. `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && ros2 daemon stop` |
| Pre-build fails with a Docker permission error | `sudo usermod -aG docker $USER`, then log out and back in |
| `Permission denied` on `/dev/ttyACM0` | `sudo usermod -aG dialout $USER`, then log out and back in |
| `_gettimeofday is not implemented` warning | Add the stub below to [`Core/Src/syscalls.c`](../uros_example/Core/Src/syscalls.c). The template already has it |

```c
int _gettimeofday_r(struct _reent *ptr, struct timeval *tv, void *tz) {
    (void)ptr;
    (void)tv;
    (void)tz;
    errno = ENOSYS;
    return -1;
}
```

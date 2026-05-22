# Bug Report: Gazebo Resource Hook Ordering

## Summary

Recent debugging on the AgriMate simulation workspace exposed an environment
hook issue affecting Gazebo resource discovery. The symptom was that Gazebo
could not find simulation assets such as worlds or models even though the
package was installed correctly and the workspace had been sourced.

The root cause was not a generic failure of `cd` inside the hook. The real
problem was the order in which the package hook and `ament_prefix_path` were
executed during environment setup.

## Symptoms

The issue appears as one or more of the following:

- Gazebo cannot find worlds, models, or related resources after
  `source install/setup.bash`.
- `GZ_SIM_RESOURCE_PATH`, `IGN_GAZEBO_RESOURCE_PATH`, or
  `GAZEBO_RESOURCE_PATH` do not contain the simulation package paths.
- The package is present in `AMENT_PREFIX_PATH`, but its resource hook does not
  contribute any paths to Gazebo.

Typical check:

```bash
env | grep -E 'GZ_SIM_RESOURCE_PATH|IGN_GAZEBO_RESOURCE_PATH|GAZEBO_RESOURCE_PATH|AMENT_PREFIX_PATH'
```

## Original Hypothesis

At first, the failure looked related to directory resolution inside the hook,
especially in fragments such as:

```sh
echo "$(CDPATH= cd -- "$(dirname "$1")" 2>/dev/null && pwd)"

_hook_dir="$(CDPATH= cd -- "$(dirname "$0")" 2>/dev/null && pwd)"
_self_prefix="$(CDPATH= cd -- "$_hook_dir/../../.." 2>/dev/null && pwd)"
```

An initial workaround was to force the shell builtin through `command cd`:

```sh
echo "$(CDPATH= command cd -- "$(dirname "$1")" 2>/dev/null && pwd)"

_hook_dir="$(CDPATH= command cd -- "$(dirname "$0")" 2>/dev/null && pwd)"
_self_prefix="$(CDPATH= command cd -- "$_hook_dir/../../.." 2>/dev/null && pwd)"
```

While this is harmless and can make shell behavior more explicit, it was not
the main cause of the AgriMate failure.

This hypothesis appeared to work during the first tests because of the way the
environment was being initialized across repeated runs:

1. On the first launch attempt, the hook failed because the package prefix was
   not yet available at the right moment.
2. That same attempt still sourced parts of the environment, including
   `ament_prefix_path`.
3. On the second launch attempt, the prefix was already present in the shell
   state.
4. The hook then appeared to work, creating the impression that the
   `command cd` change had fixed the issue.

In practice, this was a false positive caused by a partially initialized shell
environment between attempts. The apparent improvement came from the second
execution inheriting a better environment state, not from the `command cd`
modification itself.

## Real Root Cause

The hook logic depends on the package prefix being discoverable when the hook
runs. In practice, this means the package hook must run after
`ament_prefix_path` has already added the package prefix to `AMENT_PREFIX_PATH`.

The previous package name, `agrimate_simulation`, produced a hook file named
`agrimate_simulation.sh`. During environment setup this hook could be executed
before `ament_prefix_path.sh`, depending on the generated ordering.

That ordering caused this sequence:

1. The package hook executed first.
2. The package prefix was not yet present in `AMENT_PREFIX_PATH`.
3. The hook could not reliably discover its own installed prefix.
4. Gazebo resource variables were left incomplete.

By contrast, a hook whose filename sorts after `ament_prefix_path.sh` runs only
after the prefix has been added, which allows the hook to discover:

- `.../share/<package>`
- `.../share`
- `.../share/<package>/models`

and prepend those locations to:

- `GZ_SIM_RESOURCE_PATH`
- `IGN_GAZEBO_RESOURCE_PATH`
- `GAZEBO_RESOURCE_PATH`

## Why Package Hooks Must Run After `ament_prefix_path`

This class of hook inspects `AMENT_PREFIX_PATH` to locate installed packages.
If the package prefix is not yet present there, the hook has no stable source
of truth for its own location.

Therefore, hooks that derive Gazebo resource paths from installed package
prefixes should always run after `ament_prefix_path`.

This is not just a style choice. It is required for deterministic resource
resolution in generated ROS 2 environments.

## Adopted Fix

The simulation package was renamed from:

```text
agrimate_simulation
```

to:

```text
robotnik_agrimate_simulation
```

This gives the installed hook a filename that sorts after
`ament_prefix_path.sh`, making the setup order stable without relying on
temporary prefixes such as `zz_`.

The hook now lives as:

```text
hooks/robotnik_agrimate_simulation.sh.in
```

## Why This Fix Was Preferred

- It documents intent through the package name itself.
- It avoids relying on artificial ordering hacks such as `zz_`.
- It preserves a standard hook layout.
- It makes the workspace behavior reproducible after clean builds.

## Verification

After building and sourcing the workspace, the following variables must contain
the simulation package paths:

```bash
env | grep -E 'GZ_SIM_RESOURCE_PATH|IGN_GAZEBO_RESOURCE_PATH|GAZEBO_RESOURCE_PATH'
```

Expected relevant fragments include:

```text
.../install/robotnik_agrimate_simulation/share
.../install/robotnik_agrimate_simulation/share/robotnik_agrimate_simulation/models
```

It is also useful to confirm that the new package name resolves correctly:

```bash
ros2 pkg prefix robotnik_agrimate_simulation
```

## Recommendation For Future Packages

For any package that installs Gazebo resource hooks based on `AMENT_PREFIX_PATH`:

- ensure the hook is executed after `ament_prefix_path`
- avoid package or hook names that sort before `ament_prefix_path`
- prefer stable naming over ad hoc ordering tricks
- if debugging similar issues, inspect the generated environment traces before
  modifying shell builtins

## Related Files

- [README.md](../README.md)
- [hooks/robotnik_agrimate_simulation.sh.in](../hooks/robotnik_agrimate_simulation.sh.in)
- [launch/simulation.launch.py](../launch/simulation.launch.py)

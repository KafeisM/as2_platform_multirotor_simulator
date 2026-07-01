# GitHub Fork Preparation Checklist

This local overlay path is currently treated as the source of truth for the
`as2_platform_multirotor_simulator` fork used by `rl_uav_aerostack2`:

```text
/home/jordi/as2_rl_ws/src/as2_platform_multirotor_simulator
```

## Current status

- Local path: `/home/jordi/as2_rl_ws/src/as2_platform_multirotor_simulator`
- Source base: `ros-humble-as2-platform-multirotor-simulator` `1.1.3-1jammy`
- Intended remote: `<REMOTE_URL>` (choose manually before publishing)
- Intended branch: `reset-simulator-state-contract`
- Commit pin: pending until the manual fork commit exists; record with
  `git rev-parse HEAD` after review.
- Rollback note: checkout the previous known-good fork commit, or rebuild the
  base Debian source package pinned in `OVERLAY_PIN.md` while no fork commit
  exists.
- Remote: not configured by automation
- Git status: not a Git repository at the time this checklist was written
- Automation rule: Do not push from automation and do not create a remote from
  automation.

## Validation environment

Use the project runtime environment and the ROS-integrated AS2 Multirotor Simulator
overlay only:

```bash
conda activate rl_uav
source /opt/ros/humble/setup.bash
source /home/jordi/as2_rl_ws/install/setup.bash
ros2 pkg prefix as2_platform_multirotor_simulator
```

Equivalent non-interactive validator commands should use `conda run -n rl_uav`.
Gazebo and ROS-free fixed-dt simulator bindings are not acceptance paths for this
fork preparation checklist.

## Manual repository steps

Run these manually when you are ready to publish the fork:

```bash
cd /home/jordi/as2_rl_ws/src/as2_platform_multirotor_simulator
git init
git checkout -b reset-simulator-state-contract
git status --short --branch
git add .
git commit -m "feat(reset): expose simulator reset artifact metrics"
git remote add origin <REMOTE_URL>
git rev-parse HEAD
```

Only after reviewing the commit and remote target manually:

```bash
git push -u origin reset-simulator-state-contract
```

## Validation report pin

Every live validation report that relies on this fork should record:

- package prefix from `ros2 pkg prefix as2_platform_multirotor_simulator`
- branch from `git branch --show-current`
- commit pin from `git rev-parse HEAD`
- remote URL from `git remote get-url origin`
- rollback note: checkout the previous known-good commit or rebuild the base
  Debian source package pinned in `OVERLAY_PIN.md`

## Rollback

If reset behavior regresses, rollback is:

```bash
cd /home/jordi/as2_rl_ws/src/as2_platform_multirotor_simulator
git checkout <PREVIOUS_GOOD_COMMIT>
cd /home/jordi/as2_rl_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select as2_platform_multirotor_simulator
```

If no fork commit is available yet, rebuild from the pinned Debian source
artifacts listed in `OVERLAY_PIN.md`.

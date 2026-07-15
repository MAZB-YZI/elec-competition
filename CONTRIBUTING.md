# Collaboration Guide

## Golden Rule

Do not develop directly on `main`, and never force-push `main`.

Each task must use its own branch and enter `main` through a pull request.

## Start A Task

```text
git switch main
git pull --ff-only origin main
git switch -c feature/<task-name>
```

Use `fix/`, `docs/`, `test/`, or `refactor/` when those names better describe
the task.

## Commit And Push

Review the working tree before staging:

```text
git status
git diff
```

Stage only the intended files. Do not use `git add .`.

```text
git add <specific-files>
git commit -m "feat: describe the change"
git push -u origin <branch-name>
```

## Pull Requests

Open a pull request from the task branch into `main`. Include:

- what changed and why
- build and test results
- board and hardware involved
- wiring and pin changes
- `.syscfg`, PID, or protocol changes
- known risks and untested behavior

At least one teammate must review the pull request before merge.

## Repository Safety

- Never run `git push --force` or `git push --force-with-lease` on `main`.
- Never initialize a new Git repository inside this repository.
- Never replace the configured `origin` without team approval.
- Never discard another person's local changes.
- Do not commit `Resources/`, `Debug/`, `Release/`, generated binaries, logs,
  caches, secrets, or personal IDE files.
- If local and remote histories have no common ancestor, stop immediately and
  ask the repository owner. Do not pull, merge, or force-push.

## After Merge

```text
git switch main
git pull --ff-only origin main
git branch -d <merged-branch>
```

The repository owner controls releases, branch protection, and emergency
history recovery.

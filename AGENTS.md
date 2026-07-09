# Team Development Rules

These rules apply to every human and AI agent working in this repository.
More specific `AGENTS.md` files under subdirectories add extra requirements.

## Before Any Work

1. Run `git status --short --branch`.
2. Identify the current branch and inspect existing local changes.
3. Never discard, overwrite, stash, or rewrite changes that were not created
   during the current task.
4. If the worktree is dirty or `main` is behind the remote, explain the state
   before pulling, rebasing, switching branches, or creating commits.

## Branch Workflow

- Treat `main` as stable and releasable.
- Do not implement features or fixes directly on `main`.
- Start each task from an up-to-date, clean `main`:

```text
git switch main
git pull --ff-only origin main
git switch -c <type>/<short-task-name>
```

- Use one branch per task.
- Branch prefixes:
  - `feature/` for new functionality
  - `fix/` for bug fixes
  - `docs/` for documentation
  - `test/` for tests and hardware validation
  - `refactor/` for behavior-preserving restructuring

## Commits

- Stage only intentional files. Do not use `git add .`.
- Review `git diff` and `git diff --cached` before committing.
- Use focused commits with these prefixes:
  - `feat:`
  - `fix:`
  - `docs:`
  - `test:`
  - `refactor:`
  - `chore:`
- Do not commit secrets, credentials, personal IDE state, or machine-specific
  absolute paths.
- Do not commit generated output or large local resources, including:
  - `Resources/`
  - `Debug/`
  - `Release/`
  - `*.out`, `*.map`, logs, caches, and user-specific IDE files

## Hardware And CCS Changes

- CCS projects must use ASCII-only project names and paths.
- Commit source files, project metadata, linker files, and required `.syscfg`
  files when they are intentionally changed.
- Follow the nested `MSPM0G3507/AGENTS.md` before any TI/CCS task.
- Never claim hardware validation unless the code was actually built, flashed,
  and tested on the named board.
- Record board model, wiring, pins, supply voltage, test procedure, and result
  in the pull request when hardware behavior changes.

## Push And Pull Requests

- Never force-push shared branches.
- Do not push, merge, close, or delete a remote branch without explicit user
  approval.
- Push the task branch, then open a pull request into `main`.
- Every pull request must state:
  - what changed
  - why it changed
  - how it was tested
  - hardware and wiring involved
  - whether pins, `.syscfg`, PID values, or protocols changed
  - remaining risks or untested behavior
- At least one teammate should review the pull request before merge.

## After Merge

```text
git switch main
git pull --ff-only origin main
git branch -d <merged-branch>
```

If a command would lose work, rewrite shared history, or create uncertainty
about ownership of changes, stop and ask the user before running it.

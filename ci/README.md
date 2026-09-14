# CI

`github-actions.yml` is the workflow. Copy it into place and commit it:

```bash
mkdir -p .github/workflows && cp ci/github-actions.yml .github/workflows/ci.yml
```

It lives here rather than under `.github/workflows/` because the token this repository was
created with does not carry the `workflow` OAuth scope, and GitHub refuses any push that
adds or changes a workflow file without it. Nothing about the workflow itself needs
changing — it is the file, at the wrong path, waiting for a push from an account that has
the scope.

## What it runs

| Job | Command | Always? |
|---|---|---|
| Host tests | `make -C host` and `python3 scripts/test_install.py` | yes |
| Formatting | `clang-format --dry-run -Werror` over `board/` and `host/` | yes |
| ESP-IDF build | `python scripts/build.py nilo-robot` in `espressif/idf:v6.1` | only when `FIRMWARE_REPO` is set |

The firmware build clones the firmware from the repository variable `FIRMWARE_REPO`
(Settings → Secrets and variables → Actions → Variables), installs this board into it with
`scripts/install.py`, and builds. The variable exists so this repository does not hardcode
where the firmware lives; with it unset the job is skipped and the rest of CI still runs.

# Contributing

We welcome bug fixes, documentation improvements, and extensions to the 6/4 switched reluctance motor model and validation suite.

## Setup

Install Python 3.11 or later and [`uv`](https://docs.astral.sh/uv/), then sync the locked development environment:

```bash
uv sync --locked --all-groups
```

## Lint, test, and run checks

CI runs these exact commands on every pull request targeting `main`:

```bash
uv run ruff check .
uv run ruff format --check .
uv run pytest -v
uv run python validation/srm_validation.py --all
```

Keep `src/SRM_params.m` and `validation/srm_validation.py` in sync whenever machine or controller parameters change. `tests/test_srm_validation.py` verifies parameter consistency automatically.

## Compiling the paper

Run `latexmk` from `paper/` with TeX Live 2025 (or the pinned Docker image used in CI):

```bash
docker run --rm --network=none -u "$(id -u):$(id -g)" -v "$PWD:/work" -w /work/paper \
  texlive/texlive:TL2025-historic@sha256:7056390eac37715fd3e01875037ceb4016cb17d44c611f7bbbfa7a012007979f \
  latexmk -pdf -no-shell-escape -interaction=nonstopmode -halt-on-error main.tex
```

Copy `paper/main.pdf` to `docs/SRM_Speed_Torque_Control_Paper.pdf` when updating the manuscript, and remove LaTeX auxiliary files before committing.

## Pull requests and commits

1. Branch from `main` using a descriptive name (`fix/lut-boundary`, `feat/angle-advance`, `docs/update-setup`).
2. Write commit messages using [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) (`feat(scope): summary`, `fix(scope): summary`, `docs(scope): summary`, `test(scope): summary`, `chore(scope): summary`).
3. Open a pull request against `main` and fill out `.github/pull_request_template.md`.
4. Do not commit secrets, local paths, or unverified metrics.

By contributing, you agree that your contributions are licensed under the [MIT License](LICENSE).

# Contributing

**Course project — MCTR 908 Electric Drives · German University in Cairo · Spring 2026**
**Team 13:** Ahmed Mostafa (55-1591) · Andrew Abdelmalak (55-22771) · Adham Bassem (55-21599) · Ahmed Mansour (55-0253)

This repository is a graded academic submission. Contributions are limited to the four registered team members. External pull requests are not accepted.

---

## Branching

Work on feature branches off `main`. Branch naming: `type/short-description`

```
feat/tc6-advanced-angle
fix/inductance-lut-boundary
docs/update-readme
```

Merge to `main` via pull request with at least one team-member review.

## Commit Messages

Follow [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/):

```
feat(src): add TC6 advance-angle test
fix(paper): correct TC3 steady-state current value
docs(readme): update parameter table with new Imax
chore: regenerate all results figures
```

- Subject line ≤ 50 characters, imperative mood
- Blank line between subject and body
- Body explains *why*, not *what*
- `BREAKING CHANGE:` footer for any change that alters the authoritative parameter set

## File Roles

| Path | Owner | Notes |
|---|---|---|
| `src/SRM_params.m` | All | **Single source of truth** for all parameters — edit together |
| `src/SRM_Project.slx` | MATLAB users | Keep in sync with SRM_params.m |
| `validation/srm_validation.py` | Python users | Must match SRM_params.m parameter values |
| `paper/main.tex` | All | IEEE paper — no placeholder text in main |
| `results/` | Auto-generated | Run `python validation/srm_validation.py --all` to regenerate |

## Running the simulation

```bash
# MATLAB/Simulink
run('src/SRM_params.m'); sim('src/SRM_Project.slx')

# Python validation (all 5 test cases)
pip install -r requirements.txt
python validation/srm_validation.py --all
```

## Updating the paper figures

1. Run `python validation/srm_validation.py --all`
2. Copy updated PDFs from `results/` to `paper/`
3. Commit: `chore(results): regenerate TC figures`

## Code style

- MATLAB: no magic numbers — all constants come from `SRM_params.m`
- Python: PEP 8; constants in SCREAMING_SNAKE_CASE at module top
- LaTeX: one sentence per line; comment every major block

## Releases

Tag each milestone submission:

```
git tag -a v0.X.0 -m "MilestoneN — MCTR 908"
```

Current release: **v1.1.0** (post-submission review, 2026-06-23)

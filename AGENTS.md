# Konnect repository instructions

- This repository is the J3Techs fork. All branches, tags and pull requests must be published only to `J3Techs/Konnect`; use its current `main` as the default base for independent changes.
- Never push any branch, tag or other ref to `mixelpixx/Konnect`, and never create or reopen a pull request against that repository. It is a read-only reference, not a publication destination.
- Before every push, inspect the selected remote's push URL and require it to resolve to `J3Techs/Konnect`. Before publishing a PR, verify that its base repository is `J3Techs/Konnect`.
- Pass `--repo J3Techs/Konnect` explicitly to GitHub CLI PR commands. Configure the checkout's CLI default with `gh repo set-default J3Techs/Konnect`; do not rely on automatic fork-parent selection.
- These repository instructions override upstream contribution examples about remotes, base repositories and publication. See [the branch and pull request workflow](docs/BRANCH_AND_PULL_REQUEST_WORKFLOW.md).
- Keep each PR focused on its own changes. Leave merging and auto-merge disabled unless the user explicitly requests them.
- Do not include assistant or tool attribution in commits, branch names, PRs or other source-control artifacts.

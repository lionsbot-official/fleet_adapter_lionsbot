# CHANGELOG

## v2.0.1 (2026-10-23)

### Bug Fixes

- fix(SV-9512): the Docker image now installs the patched Ubuntu
  `ca-certificates` (20260816ubuntu0.24.04.1 or later), so the adapter no
  longer trusts the compromised eMudhra roots "emSign ECC Root CA - G3" and
  "AAA Certificate Services". The image build fails if the CA bundle still
  holds either root. Rebuild deployed images to take the fix. A host that runs
  the adapter without Docker uses its own CA bundle: run
  `sudo apt-get install --only-upgrade ca-certificates` on it.


## v2.0.0 (2026-08-17)

### Breaking Changes

- **LeoScrub robots are no longer supported.** The `fleet_adapter_leoscrub`
  package has been removed. Deployments running LeoScrub fleets should remain
  on v1.1.2.
- **The adapter is now a single `fleet_adapter` package.** It replaces the
  former `fleet_adapter_r3` package, so any installation or launch
  configuration referring to `fleet_adapter_r3` by name must be updated.
- **ROS 2 Jazzy is now required** (previously Foxy/Galactic), which in turn
  requires Ubuntu 24.04.

### Features

- Support for R3 (Vac, Scrub Pro) and R5 robot families. REX support is in
  testing.
- Docker Compose deployment and an RViz visualisation stack.
- A documented configuration template (`configs/config_template.yaml`) plus a
  worked `office_new` example.

### Bug Fixes

- Detect silent robot telemetry WebSocket failures and reconnect both sockets
  so robot status and pose subscriptions are restored automatically.


## v1.1.1 (2025-03-28)

### Bug Fixes

- Remove jwt token decode to obtain jwt expiry time
  ([`d680841`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/d680841acc29d184c9cde09d42fb89eaf0cf457d))


## v1.1.0 (2025-03-28)

### Features

- Add github workflow to increment release versions
  ([`620ae18`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/620ae18cdea46a42dbd95ad2a8ff500ad3a363da))


## v1.0.0 (2025-03-12)

### Bug Fixes

- Treat navigation as success when robot is near target position
  ([`5ef78ea`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/5ef78ea60606573a9da4095b7b2b9c9f91b75ecc))

### Chores

- Remove unnecessary file
  ([`a9cf007`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/a9cf007b0eec24ceb7774e9869ca438a9a6113c6))

### Features

- Add support for scrubpro and vac robot types
  ([`ab8fc1d`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/ab8fc1df715646f41b600908c473a2c4e19ca633))

- Initialize repo
  ([`a088cef`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/a088ceff4d16b25425efe693ffae536bff487bb8))

- Update broken readme url
  ([`f4f8afd`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/f4f8afd73009d45bd559daede865b7088538e7f3))

# Codex usage notes

This repo is structured so Codex can:
- see hardware assumptions (`docs/`)
- see configuration points (`include/config.h`)
- understand event/task structure (`src/main.cpp`)

Good Codex tasks:
- harden SMS parsing (handle multi-line and multiple messages)
- add modem re-init + backoff strategy
- add battery measurement and include in STATUS reply
- add optional periodic GPS tracking after carry-away

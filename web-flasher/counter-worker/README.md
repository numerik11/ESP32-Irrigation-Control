# Firmware update counter

This Cloudflare Worker records one anonymous, boot-confirmed update event per
firmware installation and displays totals on the GitHub Pages updater.

The firmware generates a random event ID for each firmware version and retains
it locally only to make retries idempotent. The service stores that opaque ID,
the firmware version, board family, and report time. It does not receive or
store a MAC address, serial number, controller configuration, or location.

## Endpoints

- `GET /v1/count?version=2.4` returns the successful-update total.
- `POST /v1/report` accepts `version`, `board`, and `eventId` after a stable boot.
- `GET /health` provides a deployment health check.

The SQLite-backed Durable Object is created by migration `v1` in
`wrangler.jsonc`. Deploy with `npx wrangler deploy` from this directory.

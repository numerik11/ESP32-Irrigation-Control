# Unit tests

Run the complete host-side suite from the repository root:

```sh
npm --prefix tests test
```

The suite has no third-party dependencies and uses Node's built-in test runner
(Node 20 or newer).  It covers pure logic extracted directly from both Arduino
sketches plus the Cloudflare update counter's request validation, CORS, routing,
and idempotent storage behavior. It also checks that every web-flasher image has
two non-overlapping OTA application slots and that its firmware fits each slot.

Hardware drivers, GPIO electrical behavior, Wi-Fi connectivity, filesystem
persistence, display rendering, and live weather requests still require an
ESP32/ESP8266 integration test on the target hardware.

The ESP32 source-contract tests also verify that Browser OTA requires credentials,
uses the application OTA partition, stops active zones before writing, and only
restarts after a complete validated upload. They do not replace a real upload test
on each supported board.

Repository contract tests enforce Arduino IDE-compatible sketch directories and
keep the documented GitHub Pages URL, worker CORS configuration, deployment files,
verification scripts, and update-counter endpoints aligned. GitHub Actions also
compiles the ESP32, ESP32-S3, and ESP8266 sketches with their pinned board cores
and library dependencies.

import assert from "node:assert/strict";
import { readFile, stat } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

const testDirectory = path.dirname(fileURLToPath(import.meta.url));
const repositoryRoot = path.resolve(testDirectory, "..");
const webFlasherDirectory = path.join(repositoryRoot, "web-flasher");

const targets = [
  {
    name: "esp32-dev",
    chipFamily: "ESP32",
    offsets: {
      "bootloader.bin": 0x1000,
      "partitions.bin": 0x8000,
      "boot_app0.bin": 0xe000,
      "firmware.bin": 0x10000,
    },
  },
  {
    name: "esp32-s3-devkitc-1",
    chipFamily: "ESP32-S3",
    offsets: {
      "bootloader.bin": 0,
      "partitions.bin": 0x8000,
      "boot_app0.bin": 0xe000,
      "firmware.bin": 0x10000,
    },
  },
];

function localPartPath(part) {
  return part.path.split("?", 1)[0];
}

async function readManifest(target) {
  const manifestPath = path.join(webFlasherDirectory, target, "manifest.json");
  return JSON.parse(await readFile(manifestPath, "utf8"));
}

for (const target of targets) {
  test(`${target.name} manifest has the expected flash image`, async () => {
    const manifest = await readManifest(target.name);
    assert.equal(manifest.builds.length, 1);
    assert.equal(manifest.builds[0].chipFamily, target.chipFamily);

    const parts = manifest.builds[0].parts;
    const paths = parts.map(localPartPath);
    assert.deepEqual(paths.sort(), Object.keys(target.offsets).sort());

    for (const part of parts) {
      const localPath = localPartPath(part);
      assert.equal(part.offset, target.offsets[localPath], `${localPath} flash offset`);

      const file = await stat(path.join(webFlasherDirectory, target.name, localPath));
      assert.ok(file.isFile(), `${part.path} references a local file`);
    }
  });
}

test("web flasher publishes one version and links both manifests", async () => {
  const manifests = await Promise.all(targets.map((target) => readManifest(target.name)));
  assert.equal(manifests[0].version, manifests[1].version, "target versions must match");

  const index = await readFile(path.join(webFlasherDirectory, "index.html"), "utf8");
  for (const target of targets) {
    assert.ok(
      index.includes(`${target.name}/manifest.json`),
      `index.html references the ${target.name} manifest`,
    );
  }

  const updaterVersion = /const\s+updaterVersion\s*=\s*["']([^"']+)["']/.exec(index);
  assert.ok(updaterVersion, "index.html declares updaterVersion");
  assert.equal(updaterVersion[1], manifests[0].version);
  const firmware = await readFile(
    path.join(repositoryRoot, "firmware", "ESP32-Irrigation", "ESP32-Irrigation.ino"),
    "utf8",
  );
  const firmwareVersion = /kFirmwareVersion\[\]\s*=\s*"([^"]+)"/.exec(firmware);
  assert.ok(firmwareVersion, "firmware declares its version");
  assert.equal(firmwareVersion[1], manifests[0].version, "firmware and updater versions match");
});

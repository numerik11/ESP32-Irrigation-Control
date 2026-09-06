import assert from "node:assert/strict";
import { readFile, readdir, stat } from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

const testDirectory = path.dirname(fileURLToPath(import.meta.url));
const repositoryRoot = path.resolve(testDirectory, "..");
const firmwareDirectory = path.join(repositoryRoot, "firmware");
const webFlasherDirectory = path.join(repositoryRoot, "web-flasher");
const workerDirectory = path.join(webFlasherDirectory, "counter-worker");

async function assertFile(filePath, message) {
  const entry = await stat(filePath);
  assert.ok(entry.isFile(), message);
}

test("Arduino sketches use IDE-compatible same-name directories", async () => {
  const entries = await readdir(firmwareDirectory, { withFileTypes: true });
  const rootSketches = entries.filter((entry) => entry.isFile() && entry.name.endsWith(".ino"));
  assert.deepEqual(rootSketches, [], "firmware root must not contain loose .ino sketches");

  const sketchDirectories = entries.filter((entry) => entry.isDirectory());
  assert.ok(sketchDirectories.length > 0, "firmware must contain at least one sketch directory");
  for (const directory of sketchDirectories) {
    await assertFile(
      path.join(firmwareDirectory, directory.name, `${directory.name}.ino`),
      `${directory.name} must contain the primary sketch ${directory.name}.ino`,
    );
  }
});

test("GitHub Pages documentation, local path, and worker CORS origin stay aligned", async () => {
  const rootReadme = await readFile(path.join(repositoryRoot, "README.md"), "utf8");
  const flasherReadme = await readFile(path.join(webFlasherDirectory, "README.md"), "utf8");
  const workerSource = await readFile(path.join(workerDirectory, "worker.mjs"), "utf8");

  const pageUrlPattern = /https:\/\/[\w-]+\.github\.io\/[\w-]+\/web-flasher\//g;
  const documentedUrls = [...rootReadme.matchAll(pageUrlPattern), ...flasherReadme.matchAll(pageUrlPattern)]
    .map((match) => match[0]);
  assert.ok(documentedUrls.length >= 3, "the Pages URL must be documented in both READMEs");
  assert.equal(new Set(documentedUrls).size, 1, "all documented Pages URLs must match");

  const pagesUrl = new URL(documentedUrls[0]);
  assert.equal(path.basename(pagesUrl.pathname), "web-flasher");
  const flasher = await stat(webFlasherDirectory);
  assert.ok(flasher.isDirectory(), "the documented web-flasher path must exist");

  const allowedOrigin = /const\s+ALLOWED_ORIGIN\s*=\s*["']([^"']+)["']/.exec(workerSource);
  assert.ok(allowedOrigin, "the counter worker must declare an allowed origin");
  assert.equal(allowedOrigin[1], pagesUrl.origin, "worker CORS must allow the Pages origin");
});

test("worker deployment configuration references the exported Durable Object", async () => {
  const configPath = path.join(workerDirectory, "wrangler.jsonc");
  const config = JSON.parse(await readFile(configPath, "utf8"));
  const workerPath = path.join(workerDirectory, config.main);
  await assertFile(workerPath, "Wrangler's main worker file must exist");

  const workerSource = await readFile(workerPath, "utf8");
  const bindings = config.durable_objects?.bindings ?? [];
  assert.ok(bindings.length > 0, "Wrangler must configure a Durable Object binding");
  for (const binding of bindings) {
    assert.match(
      workerSource,
      new RegExp(`export\\s+class\\s+${binding.class_name}\\b`),
      `${binding.class_name} must be exported by the worker`,
    );
    assert.ok(
      config.migrations?.some((migration) =>
        migration.new_sqlite_classes?.includes(binding.class_name)),
      `${binding.class_name} must have a SQLite migration`,
    );
  }
});

test("documented scripts and update-counter endpoints stay valid", async () => {
  const rootReadme = await readFile(path.join(repositoryRoot, "README.md"), "utf8");
  const firmware = await readFile(
    path.join(firmwareDirectory, "ESP32-Irrigation", "ESP32-Irrigation.ino"),
    "utf8",
  );
  const flasher = await readFile(path.join(webFlasherDirectory, "index.html"), "utf8");

  const documentedScripts = [...rootReadme.matchAll(/\.\\([^\s`]+\.ps1)/g)]
    .map((match) => match[1].replaceAll("\\", path.sep));
  assert.ok(documentedScripts.length > 0, "README must document the verification script");
  for (const script of documentedScripts) {
    await assertFile(path.join(repositoryRoot, script), `documented script ${script} must exist`);
  }

  const firmwareCounter = /kUpdateReportUrl\[\]\s*=\s*["']([^"']+)["']/.exec(firmware);
  const flasherCounter = /const\s+counterUrl\s*=\s*["']([^"']+)["']/.exec(flasher);
  assert.ok(firmwareCounter, "firmware must declare its update-report endpoint");
  assert.ok(flasherCounter, "web flasher must declare its update-counter endpoint");
  const firmwareUrl = new URL(firmwareCounter[1]);
  const flasherUrl = new URL(flasherCounter[1]);
  assert.equal(firmwareUrl.origin, flasherUrl.origin, "firmware and flasher must use one worker");
  assert.equal(firmwareUrl.pathname, "/v1/report");
});

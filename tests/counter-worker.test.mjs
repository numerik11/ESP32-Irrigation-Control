import assert from "node:assert/strict";
import path from "node:path";
import { fileURLToPath } from "node:url";
import test from "node:test";

import {
  loadWorkerModule,
  makeCounterEnvironment,
  responseJson,
} from "./helpers/load-worker.mjs";

const testDirectory = path.dirname(fileURLToPath(import.meta.url));
const workerPath = path.resolve(testDirectory, "../web-flasher/counter-worker/worker.mjs");
const workerModule = await loadWorkerModule(workerPath);
const worker = workerModule.default;

test("health endpoint responds without using durable storage", async () => {
  const { env, calls } = makeCounterEnvironment();
  const result = await responseJson(
    await worker.fetch(new Request("https://worker.example/health"), env),
  );

  assert.equal(result.status, 200);
  assert.deepEqual(result.body, { ok: true });
  assert.equal(result.headers.get("Cache-Control"), "no-store");
  assert.deepEqual(calls, []);
});

test("count endpoint validates and trims firmware versions", async () => {
  const { env, calls } = makeCounterEnvironment({ count: async () => 42 });
  const result = await responseJson(
    await worker.fetch(new Request("https://worker.example/v1/count?version=%202.4.1%20"), env),
  );

  assert.equal(result.status, 200);
  assert.deepEqual(result.body, { version: "2.4.1", count: 42 });
  assert.deepEqual(calls, [{ name: "2.4.1", options: { locationHint: "oc" } }]);

  for (const version of ["", "v2.4", "2", "2.4.1.5", "2.4-beta"]) {
    const invalid = await responseJson(
      await worker.fetch(
        new Request(`https://worker.example/v1/count?version=${encodeURIComponent(version)}`),
        env,
      ),
    );
    assert.equal(invalid.status, 400, version);
    assert.deepEqual(invalid.body, { error: "Invalid version" });
  }
});

test("report endpoint accepts a valid board event and normalizes its ID", async () => {
  const reports = [];
  const { env, calls } = makeCounterEnvironment({
    report: async (board, eventId) => {
      reports.push({ board, eventId });
      return { count: 9, recorded: true };
    },
  });
  const body = JSON.stringify({
    version: "2.4",
    board: "esp32-dev",
    eventId: "ABCDEF0123456789ABCDEF0123456789",
  });
  const request = new Request("https://worker.example/v1/report", {
    method: "POST",
    headers: {
      "Content-Type": "application/json; charset=utf-8",
      "Content-Length": String(Buffer.byteLength(body)),
    },
    body,
  });
  const result = await responseJson(await worker.fetch(request, env));

  assert.equal(result.status, 200);
  assert.deepEqual(result.body, { version: "2.4", count: 9, recorded: true });
  assert.deepEqual(reports, [{
    board: "esp32-dev",
    eventId: "abcdef0123456789abcdef0123456789",
  }]);
  assert.equal(calls[0].name, "2.4");
});

test("report endpoint rejects unsafe request shapes and invalid data", async () => {
  const { env, calls } = makeCounterEnvironment();

  const missingLength = await responseJson(
    await worker.fetch(new Request("https://worker.example/v1/report", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: "{}",
    }), env),
  );
  assert.equal(missingLength.status, 400);

  const wrongTypeBody = "{}";
  const wrongType = await responseJson(
    await worker.fetch(new Request("https://worker.example/v1/report", {
      method: "POST",
      headers: {
        "Content-Type": "text/plain",
        "Content-Length": String(wrongTypeBody.length),
      },
      body: wrongTypeBody,
    }), env),
  );
  assert.equal(wrongType.status, 415);

  for (const payload of [
    { version: "v2.4", board: "esp32-dev", eventId: "a".repeat(32) },
    { version: "2.4", board: "unknown-board", eventId: "a".repeat(32) },
    { version: "2.4", board: "esp32-dev", eventId: "not-an-event-id" },
  ]) {
    const body = JSON.stringify(payload);
    const result = await responseJson(
      await worker.fetch(new Request("https://worker.example/v1/report", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          "Content-Length": String(Buffer.byteLength(body)),
        },
        body,
      }), env),
    );
    assert.equal(result.status, 400);
    assert.deepEqual(result.body, { error: "Invalid report" });
  }
  assert.deepEqual(calls, []);
});

test("CORS is granted only to the GitHub Pages updater", async () => {
  const { env } = makeCounterEnvironment();
  const allowed = await worker.fetch(new Request("https://worker.example/health", {
    headers: { Origin: "https://numerik11.github.io" },
  }), env);
  assert.equal(allowed.headers.get("Access-Control-Allow-Origin"), "https://numerik11.github.io");
  assert.equal(allowed.headers.get("Vary"), "Origin");

  const denied = await worker.fetch(new Request("https://worker.example/health", {
    headers: { Origin: "https://attacker.example" },
  }), env);
  assert.equal(denied.headers.get("Access-Control-Allow-Origin"), null);

  const preflight = await worker.fetch(new Request("https://worker.example/v1/report", {
    method: "OPTIONS",
    headers: { Origin: "https://numerik11.github.io" },
  }), env);
  assert.equal(preflight.status, 204);
  assert.equal(preflight.headers.get("Access-Control-Allow-Methods"), "GET, POST, OPTIONS");
});

test("unknown routes return a JSON 404", async () => {
  const { env } = makeCounterEnvironment();
  const result = await responseJson(
    await worker.fetch(new Request("https://worker.example/not-found"), env),
  );
  assert.equal(result.status, 404);
  assert.deepEqual(result.body, { error: "Not found" });
});

test("UpdateCounter records each event ID at most once", () => {
  const reports = new Map();
  const sql = {
    exec(statement, ...parameters) {
      if (statement.includes("INSERT OR IGNORE")) {
        const [eventId, board, reportedAt] = parameters;
        if (reports.has(eventId)) return { toArray: () => [] };
        reports.set(eventId, { board, reportedAt });
        return { toArray: () => [{ event_id: eventId }] };
      }
      if (statement.includes("SELECT COUNT")) {
        return { one: () => ({ count: reports.size }) };
      }
      return { toArray: () => [] };
    },
  };
  const ctx = {
    storage: { sql },
    blockConcurrencyWhile(callback) {
      return callback();
    },
  };
  const counter = new workerModule.UpdateCounter(ctx, {});

  assert.deepEqual(counter.report("esp32-dev", "a".repeat(32)), { count: 1, recorded: true });
  assert.deepEqual(counter.report("esp32-dev", "a".repeat(32)), { count: 1, recorded: false });
  assert.deepEqual(counter.report("esp32-s3-devkitc-1", "b".repeat(32)), { count: 2, recorded: true });
  assert.equal(counter.count(), 2);
});

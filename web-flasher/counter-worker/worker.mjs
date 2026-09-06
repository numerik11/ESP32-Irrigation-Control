import { DurableObject } from "cloudflare:workers";

const ALLOWED_ORIGIN = "https://numerik11.github.io";
const VERSION_PATTERN = /^\d+\.\d+(?:\.\d+)?$/;
const EVENT_ID_PATTERN = /^[a-f0-9]{32}$/;
const VALID_BOARDS = new Set(["esp32-dev", "esp32-s3-devkitc-1"]);

function corsHeaders(request) {
  return request.headers.get("Origin") === ALLOWED_ORIGIN
    ? {
        "Access-Control-Allow-Origin": ALLOWED_ORIGIN,
        "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
        Vary: "Origin",
      }
    : {};
}

function jsonResponse(request, body, status = 200) {
  return Response.json(body, {
    status,
    headers: {
      ...corsHeaders(request),
      "Cache-Control": "no-store",
      "X-Content-Type-Options": "nosniff",
    },
  });
}

export class UpdateCounter extends DurableObject {
  constructor(ctx, env) {
    super(ctx, env);
    ctx.blockConcurrencyWhile(async () => {
      this.ctx.storage.sql.exec(`
        CREATE TABLE IF NOT EXISTS reports (
          event_id TEXT PRIMARY KEY,
          board TEXT NOT NULL,
          reported_at INTEGER NOT NULL
        );
        CREATE INDEX IF NOT EXISTS reports_board_idx ON reports(board);
      `);
    });
  }

  report(board, eventId) {
    const inserted = this.ctx.storage.sql.exec(
      "INSERT OR IGNORE INTO reports (event_id, board, reported_at) VALUES (?, ?, ?) RETURNING event_id",
      eventId,
      board,
      Date.now(),
    );
    const recorded = inserted.toArray().length === 1;
    const row = this.ctx.storage.sql
      .exec("SELECT COUNT(*) AS count FROM reports")
      .one();
    return { count: Number(row.count), recorded };
  }

  count() {
    const row = this.ctx.storage.sql
      .exec("SELECT COUNT(*) AS count FROM reports")
      .one();
    return Number(row.count);
  }
}

export default {
  async fetch(request, env) {
    const url = new URL(request.url);

    if (request.method === "OPTIONS") {
      return new Response(null, { status: 204, headers: corsHeaders(request) });
    }

    if (request.method === "GET" && url.pathname === "/health") {
      return jsonResponse(request, { ok: true });
    }

    const version = url.searchParams.get("version")?.trim() ?? "";
    if (request.method === "GET" && url.pathname === "/v1/count") {
      if (!VERSION_PATTERN.test(version)) {
        return jsonResponse(request, { error: "Invalid version" }, 400);
      }
      const counter = env.UPDATE_COUNTER.getByName(version, { locationHint: "oc" });
      return jsonResponse(request, { version, count: await counter.count() });
    }

    if (request.method === "POST" && url.pathname === "/v1/report") {
      const contentLength = Number(request.headers.get("Content-Length"));
      const contentType = request.headers.get("Content-Type") ?? "";
      if (!Number.isInteger(contentLength) || contentLength < 2 || contentLength > 512) {
        return jsonResponse(request, { error: "Invalid content length" }, 400);
      }
      if (!contentType.toLowerCase().startsWith("application/json")) {
        return jsonResponse(request, { error: "JSON required" }, 415);
      }

      try {
        const body = await request.json();
        const reportVersion = String(body.version ?? "").trim();
        const board = String(body.board ?? "").trim();
        const eventId = String(body.eventId ?? "").trim().toLowerCase();
        if (
          !VERSION_PATTERN.test(reportVersion) ||
          !VALID_BOARDS.has(board) ||
          !EVENT_ID_PATTERN.test(eventId)
        ) {
          return jsonResponse(request, { error: "Invalid report" }, 400);
        }

        const counter = env.UPDATE_COUNTER.getByName(reportVersion, {
          locationHint: "oc",
        });
        const result = await counter.report(board, eventId);
        return jsonResponse(request, { version: reportVersion, ...result });
      } catch (error) {
        console.error(
          JSON.stringify({
            message: "update report failed",
            error: error instanceof Error ? error.message : String(error),
            path: url.pathname,
          }),
        );
        return jsonResponse(request, { error: "Invalid JSON" }, 400);
      }
    }

    return jsonResponse(request, { error: "Not found" }, 404);
  },
};

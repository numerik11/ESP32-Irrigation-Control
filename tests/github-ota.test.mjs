import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import vm from 'node:vm';
import test from 'node:test';

const source = await readFile(new URL('../firmware/ESP32-Irrigation/ESP32-Irrigation.ino', import.meta.url), 'utf8');
const line = source.split('\n').find(line => line.includes('html += F("function githubFirmwareUrl'));
const script = JSON.parse(line.trim().slice(10, -2));
const context = vm.createContext({ URL });
vm.runInContext(script, context);

test('GitHub OTA normalizes file links and preserves raw firmware paths', () => {
  const raw = 'https://raw.githubusercontent.com/owner/repo/main/firmware.bin';
  assert.equal(context.githubFirmwareUrl(raw), raw);
  assert.equal(context.githubFirmwareUrl('https://github.com/owner/repo/blob/main/firmware.bin?raw=true'), raw);
  assert.equal(context.githubFirmwareUrl('https://github.com/owner/repo/raw/main/firmware.bin'), raw);
});

test('GitHub OTA rejects unsupported hosts, insecure URLs and non-firmware paths', () => {
  for (const url of [
    'http://github.com/owner/repo/blob/main/firmware.bin',
    'https://example.com/firmware.bin',
    'https://github.com/owner/repo/releases/download/v2/firmware.bin',
    'https://raw.githubusercontent.com/owner/repo/main/page.html',
    'https://user:password@raw.githubusercontent.com/owner/repo/main/firmware.bin',
    'not a URL',
  ]) assert.throws(() => context.githubFirmwareUrl(url), url);
});

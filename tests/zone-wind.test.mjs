import assert from 'node:assert/strict';
import test from 'node:test';
import { readFirmware, compileFirmwareFunctions, extractFunction } from './helpers/firmware-source.mjs';

const source = await readFirmware(new URL('../firmware/ESP32-Irrigation/ESP32-Irrigation.ino', import.meta.url));

test('wind only blocks enabled zones while the master wind rule is active', () => {
  for (const windActive of [false, true]) {
    const { windBlocksZone } = compileFirmwareFunctions(source, ['windBlocksZone'], {
      MAX_ZONES: 16, windActive, zoneWindDelayEnabled: [true, false, true],
    });
    assert.equal(windBlocksZone(0), windActive);
    assert.equal(windBlocksZone(1), false);
    assert.equal(windBlocksZone(2), windActive);
    assert.equal(windBlocksZone(-1), false);
    assert.equal(windBlocksZone(16), false);
  }
});

test('wind leaves exempt running zones alone; rain and pause still stop them', () => {
  for (const [rainActive, paused, expected] of [[false, false, [0]], [true, false, [0, 1]], [false, true, [0, 1]]]) {
    const stopped = [];
    const functions = compileFirmwareFunctions(source, ['windBlocksZone', 'stopAutoZonesForBlock'], {
      MAX_ZONES: 16, zonesCount: 3, windActive: true,
      zoneWindDelayEnabled: [true, false, true], zoneActive: [true, true, true],
      zoneStartedManual: [false, false, true], systemMasterEnabled: true,
      isBlockedNow: () => paused, rainActive, turnOffZone: z => stopped.push(z),
    });
    functions.stopAutoZonesForBlock();
    assert.deepEqual(stopped, expected);
  }
});

test('zone wind preference is connected to both save paths and backward-compatible storage', () => {
  assert.match(source, /name='zoneWindDelay/);
  assert.match(source, /add\('zoneWindDelay'\+z\)/);
  assert.equal((extractFunction(source, 'handleSubmit').match(/zoneWindDelayEnabled\[z\] = server.hasArg/g) || []).length, 2);
  assert.match(extractFunction(source, 'loadSchedule'), /zoneWindDelayEnabled\[i\] = tok\(16, 1\) == 1/);
  assert.match(extractFunction(source, 'saveSchedule'), /f.print\(zoneWindDelayEnabled\[i\]/);
  assert.match(extractFunction(source, 'turnOnZone'), /if \(windBlocksZone\(z\)\)/);
  const loop = extractFunction(source, 'loop');
  assert.equal((loop.match(/pendingStart\[z\] && !windBlocksZone\(z\)/g) || []).length, 2);
});

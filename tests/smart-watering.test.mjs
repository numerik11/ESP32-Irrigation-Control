import assert from 'node:assert/strict';
import test from 'node:test';
import vm from 'node:vm';
import {readFirmware, compileFirmwareFunctions, extractFunction} from './helpers/firmware-source.mjs';

const source=await readFirmware(new URL('../firmware/ESP32-Irrigation/ESP32-Irrigation.ino',import.meta.url));
const javascript=/R"SMARTJS\(([\s\S]*?)\)SMARTJS"/.exec(source)[1];
const helpers=compileFirmwareFunctions(source,['smartRuleForTemperature','smartRuleAdjustment','smartLimitRuntime']);

test('temperature boundaries, exclusive Very Hot, and hysteresis transitions',()=>{
  const rule=(t,previous=-1,h=1)=>helpers.smartRuleForTemperature(t,previous,15,30,37,h);
  assert.equal(rule(14.9),0);
  assert.equal(rule(15),1);
  assert.equal(rule(29.9),1);
  assert.equal(rule(30),2);
  assert.equal(rule(37),3);
  assert.equal(rule(29,2),2);
  assert.equal(rule(28.9,2),1);
  assert.equal(rule(36,3),3);
  assert.equal(rule(35.9,3),2);
  assert.equal(rule(15.9,0),0);
  assert.equal(rule(16,0),1);
  assert.equal(rule(10,3),0,'large downward jumps cross bands');
  assert.equal(rule(40,0),3,'large upward jumps cross bands');
  assert.equal(rule(29.9,2,0),1,'zero disables hysteresis');
  assert.equal(rule(NaN,3),-1);
  assert.equal(helpers.smartRuleAdjustment(rule(37),-50,25,50),50,'Very Hot replaces Hot');
});

test('runtime floor, cap, skips, and disabled schedules',()=>{
  const duration=helpers.smartLimitRuntime;
  assert.equal(duration(1800,1.3,5,100),2340,'30 minutes becomes 39');
  assert.equal(duration(7200,1.3,5,100),9360,'120 minutes becomes 156');
  assert.equal(duration(3600,0.5,5,100),1800);
  assert.equal(duration(300,0.5,5,100),300,'five-minute zone remains five minutes');
  assert.equal(duration(120,0.5,5,100),120,'minimum does not lengthen short schedules');
  assert.equal(duration(3600,4,5,100),7200,'combined increase is capped');
  assert.equal(duration(300,0,5,100),0,'minimum never revives a skip');
  assert.equal(duration(0,1.5,5,100),0,'disabled schedule stays zero');
});

function firmware(overrides={}){
  const state={smartWateringEnabled:true,MAX_ZONES:16,smartTempBasis:1,curTempC:12,todayMin_C:20,todayMax_C:34.2,
    smartCoolTempC:15,smartHotTempC:30,smartVeryHotTempC:37,smartHysteresisC:1,smartRuleState:-1,
    smartCoolAdjustPct:-50,smartHotAdjustPct:25,smartVeryHotAdjustPct:50,
    smartSeasonalPct:100,smartMaximumIncreasePct:100,smartMinimumMin:5,
    smartZoneMode:[0,1,2],smartZoneCoolPct:[0,0,-10],smartZoneHotPct:[0,0,30],smartZoneVeryHotPct:[0,0,60],
    rainNext24h_mm:0,smartActualRainSkipMm:5,smartForecastRainSkipMm:5,smartLightRainAdjustPct:-30,
    last24hActualRain:()=>0,isSoilWetForSmartSkip:()=>false,durationForSlot:()=>1800,
    constrain:(v,a,b)=>Math.min(b,Math.max(a,v)),...overrides};
  return compileFirmwareFunctions(source,['smartWateringReferenceTempC','smartRuleForTemperature','smartCurrentRule',
    'smartRuleAdjustment','smartFactorForZone','smartWateringFactor','smartLimitRuntime','smartWateringDurationForSlot'],state);
}

test('selected temperature source, explicit missing forecast, and legacy fallback',()=>{
  assert.equal(firmware().smartWateringReferenceTempC(),34.2);
  assert.equal(firmware({smartTempBasis:0}).smartWateringReferenceTempC(),12);
  assert.equal(firmware({smartTempBasis:2}).smartWateringReferenceTempC(),27.1);
  assert.equal(firmware({smartTempBasis:0,curTempC:NaN}).smartWateringReferenceTempC(),34.2);
  assert.ok(Number.isNaN(firmware({todayMax_C:NaN}).smartWateringReferenceTempC()));
  assert.equal(firmware({todayMax_C:NaN}).smartWateringFactor(),1);
});

test('configuration tail includes all sixteen zones and preserves legacy defaults',()=>{
  const load=extractFunction(source,'loadConfig'),save=extractFunction(source,'saveConfig');
  assert.match(load,/String tail\[64 \+ 5 \+ 4 \* MAX_ZONES\]/,'reserve room beyond the legacy tail');
  assert.match(load,/smartTempBasis=0; smartMinimumMin=0; smartMaximumIncreasePct=1500; smartHysteresisC=0; smartSeasonalPct=100;/);
  const fields=['smartTempBasis','smartMinimumMin','smartMaximumIncreasePct','smartHysteresisC','smartSeasonalPct',
    'smartZoneMode','smartZoneCoolPct','smartZoneHotPct','smartZoneVeryHotPct'];
  let readOffset=load.indexOf('// Optional v2.9 tail.'),writeOffset=save.indexOf('f.println(moistureUseMeteo');
  for(const field of fields){
    const read=load.indexOf(`) ${field}`,readOffset);
    const write=save.indexOf(`f.println(${field}`,writeOffset);
    assert.ok(read>readOffset,`${field} read order`);
    assert.ok(write>writeOffset,`${field} write order`);
    readOffset=read;writeOffset=write;
  }
  const configure=extractFunction(source,'handleConfigure');
  assert.ok(configure.indexOf('if (!smartValid)')<configure.indexOf('smartWateringEnabled ='));
  assert.match(configure,/strtof\(raw.c_str\(\), &end\)/,'reject malformed numeric input instead of coercing it to zero');
});

test('zone opt-out and custom rules, seasonal scaling, and skip priority',()=>{
  assert.equal(firmware().smartWateringDurationForSlot(0,1),2250);
  assert.equal(firmware().smartWateringDurationForSlot(1,1),1800);
  assert.equal(firmware().smartWateringDurationForSlot(2,1),2340);
  assert.equal(firmware({smartSeasonalPct:80}).smartWateringDurationForSlot(0,1),1800);
  assert.equal(firmware({smartSeasonalPct:200}).smartWateringDurationForSlot(0,1),3600);
  assert.equal(firmware({smartSeasonalPct:0}).smartWateringDurationForSlot(0,1),0);
  for(const z of [0,1,2]){
    assert.equal(firmware({isSoilWetForSmartSkip:()=>true}).smartWateringDurationForSlot(z,1),0);
    assert.equal(firmware({last24hActualRain:()=>6}).smartWateringDurationForSlot(z,1),0);
    assert.equal(firmware({rainNext24h_mm:6}).smartWateringDurationForSlot(z,1),0);
    assert.equal(firmware({smartWateringEnabled:false}).smartWateringDurationForSlot(z,1),1800);
  }
});

test('setup temperature ranges and validation update without a runtime preview',()=>{
  const setup=extractFunction(source,'handleSetupPage');
  assert.ok(!setup.includes('Live runtime preview'));
  assert.ok(!setup.includes('smartPreviewData'));
  const fields=Object.fromEntries(Object.entries({tempUnit:'C',smartCoolTemp:'15',smartHotTemp:'30',smartVeryHotTemp:'37'}).map(([name,value])=>[name,{value,setCustomValidity(message){this.error=message;}}]));
  const events={};
  const range={};
  const form={elements:{namedItem:name=>fields[name]},addEventListener:(name,fn)=>events[name]=fn};
  vm.runInNewContext(javascript,{document:{getElementById:id=>id==='setupForm'?form:range}});
  assert.equal(range.textContent,'15.0 C to below 30.0 C');
  assert.equal(fields.smartHotTemp.error,'');
  fields.smartHotTemp.value='10';events.input();
  assert.ok(fields.smartHotTemp.error);
  fields.tempUnit.value='F';fields.smartCoolTemp.value='59';fields.smartHotTemp.value='86';fields.smartVeryHotTemp.value='98.6';events.change();
  assert.equal(range.textContent,'59.0 F to below 86.0 F');
  assert.equal(fields.smartHotTemp.max,'140');
  assert.equal(fields.smartHotTemp.error,'');
  fields.smartCoolTemp.value='';events.input();
  assert.equal(range.textContent,'');
  assert.ok(fields.smartHotTemp.error);
});

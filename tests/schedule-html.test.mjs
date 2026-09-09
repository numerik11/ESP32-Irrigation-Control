import assert from 'node:assert/strict';
import test from 'node:test';
import {readFirmware,compileFirmwareFunctions} from './helpers/firmware-source.mjs';
const source=await readFirmware(new URL('../firmware/ESP32-Irrigation/ESP32-Irrigation.ino',import.meta.url));
function page(overrides={}){
  const response={headers:{}};
  const state={MAX_ZONES:3,zonesCount:3,days:[[false,false,false,true,false,false,false],[false,false,false,true,false,false,false],[true,false,false,false,false,false,false]],
    zoneNames:['Audrey <II> & herbs','Tomatoes','Other day'],startHour:[17,23,10],startMin:[30,50,0],startHour2:[11,0,0],startMin2:[30,0,0],enableStartTime2:[true,false,false],
    durationForSlot:()=>1800,smartWateringDurationForSlot:()=>1800,
    time:()=>Date.UTC(2026,8,9,18)/1000,F:v=>v,String,sizeof:()=>40,
    server:{sendHeader:(k,v)=>response.headers[k]=v,send:(code,type,html)=>Object.assign(response,{code,type,html})},
    localtime_r:(epoch,out)=>{const d=new Date(epoch*1000);Object.assign(out,{tm_year:d.getUTCFullYear()-1900,tm_mon:d.getUTCMonth(),tm_mday:d.getUTCDate(),tm_hour:d.getUTCHours(),tm_min:d.getUTCMinutes(),tm_sec:d.getUTCSeconds(),tm_wday:d.getUTCDay(),tm_yday:Math.floor((d-Date.UTC(d.getUTCFullYear(),0,1))/86400000)});return out;},
    mktime:t=>Date.UTC(t.tm_year+1900,t.tm_mon,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec)/1000,
    strftime:(out,size,format,t)=>{const pad=v=>String(v).padStart(2,'0');out.value=format==='%H:%M'?pad(t.tm_hour)+':'+pad(t.tm_min):format==='%H:%M:%S'?pad(t.tm_hour)+':'+pad(t.tm_min)+':'+pad(t.tm_sec):format==='%Y-%m-%d'?(t.tm_year+1900)+'-'+pad(t.tm_mon+1)+'-'+pad(t.tm_mday):'Wednesday, 09 September 2026';},...overrides};
  const funcs=compileFirmwareFunctions(source,['scheduleSlotToday','scheduleTimeRange','htmlEscape','handleScheduleHtml'],state,{replacements:[
    [/HttpScope _scope;/g,''],[/struct tm start = today;/g,'let start = {...today};'],[/struct tm (today|end);/g,'let $1 = {};'],[/&(start|end|today|finish|now)\b/g,'$1'],
    [/char (clock|date)\[\d+\];/g,'const $1 = {value:"",toString(){return this.value;}};'],[/String (html|out);/g,'let $1 = "";'],[/String result\(clock\);/g,'let result = String(clock);'],[/String name =/g,'let name ='],[/\b(html|out)\.reserve\([^;]+;/g,''],[/\bsize_t i/g,'let i'],[/\.length\(\)/g,'.length'],[/name.trim\(\);/g,'name = name.trim();']
  ]});
  funcs.handleScheduleHtml();return {response,funcs};
}
test('iframe schedule renders named zones, both starts in time order, past starts and next-day ends',()=>{
  const {response}=page();
  assert.equal(response.code,200);assert.equal(response.type,'text/html; charset=utf-8');
  assert.equal(response.headers['Cache-Control'],'no-store');
  assert.match(response.html,/Audrey &lt;II&gt; &amp; herbs/);
  assert.ok(response.html.indexOf('11:30 &ndash; 12:00')<response.html.indexOf('17:30 &ndash; 18:00'));
  assert.match(response.html,/23:50 &ndash; 00:20 \(2026-09-10\)/);
  assert.ok(!response.html.includes('Other day'));
  assert.match(response.html,/http-equiv='refresh' content='60'/);
  assert.ok(source.includes('server.on("/schedule-html", HTTP_GET, handleScheduleHtml)'));
});
test('schedule handles disabled slots, seconds, skipped runs, unnamed zones and empty days',()=>{
  assert.match(page({smartWateringDurationForSlot:()=>75}).response.html,/11:31:15/);
  assert.match(page({smartWateringDurationForSlot:()=>0}).response.html,/skipped \(Smart Watering\)/);
  assert.match(page({zoneNames:['  ','Tomatoes','Other']}).response.html,/>Zone 1</);
  assert.match(page({durationForSlot:()=>0}).response.html,/No watering scheduled today/);
  const {funcs}=page();
  assert.equal(funcs.scheduleSlotToday(0,3,1),true);
  for(const args of [[-1,3,1],[3,3,1],[0,-1,1],[0,7,1],[0,2,1],[1,3,2],[0,3,3]])assert.equal(funcs.scheduleSlotToday(...args),false);
  assert.equal(page({startHour:[24,23,10]}).funcs.scheduleSlotToday(0,3,1),false);
});
test('unsynchronized controller clock renders an explicit waiting message',()=>{
  const {response}=page({time:()=>0});
  assert.match(response.html,/Waiting for the controller clock/);
  assert.ok(!response.html.includes('<table>'));
});

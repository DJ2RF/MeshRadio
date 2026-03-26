#include "incl/mr_web_cmdcfg.h"

const char *mr_web_cmdcfg_html(void)
{
    return
    "<div class='card'>"
    "  <h3>Command Trigger</h3>"
    "  <div class='muted' style='margin-bottom:10px'>"
    "    Configure trigger GPIO, destination and command text."
    "  </div>"

    "  <div class='cfg-cmd-grid'>"
    "    <div>"
    "      <label>Trigger GPIO</label>"
    "      <select id='cfg_cmd_trigger_gpio'>"
    "        <option value='-1'>deaktiviert</option>"
     "        <option value='13'>LiLyGo GPIO 13</option>"
    "        <option value='14'>LiLyGo GPIO 14</option>"
    "        <option value='26'>GPIO 26</option>"
    "        <option value='48'>GPIO 48</option>"
    "      </select>"
    "    </div>"

    "    <div>"
    "      <label>Command Destination (DST)</label>"
    "      <input id='cfg_cmd_dst' maxlength='8' placeholder='z. B. DJ2RF'>"
    "    </div>"

    "    <div>"
    "      <label>Command Text</label>"
    "      <select id='cfg_cmd_text_select' onchange='cmdCfgApplyPreset()'>"
    "        <option value=''>-- Select Command --</option>"
    "        <option value='CMD:STATUS?'>STATUS</option>"
    "        <option value='CMD:DISPLAY ON'>DISPLAY ON</option>"
    "        <option value='CMD:DISPLAY OFF'>DISPLAY OFF</option>"
    "        <option value='CMD:WIFI ON'>WIFI ON</option>"
    "        <option value='CMD:WIFI OFF'>WIFI OFF</option>"
    "        <option value='CMD:GPS'>GPS</option>"
    "        <option value='CMD:BATT'>BATT</option>"
    "        <option value='CMD:VERSION'>VERSION</option>"
    "        <option value='CMD:REBOOT'>REBOOT</option>"
    "        <option value='CMD:RELAY ON'>RELAY ON</option>"
    "        <option value='CMD:RELAY OFF'>RELAY OFF</option>"
    "        <option value='CMD:RELAY TOGGLE'>RELAY TOGGLE</option>"
    "        <option value='CMD:CRYPTO ON'>CRYPTO ON</option>"
    "        <option value='CMD:CRYPTO OFF'>CRYPTO OFF</option>"
    "        <option value='CMD:BEACON ON'>BEACON ON</option>"
    "        <option value='CMD:BEACON OFF'>BEACON OFF</option>"
    "        <option value='CMD:ROUTEADV ON'>ROUTEADV ON</option>"
    "        <option value='CMD:ROUTEADV OFF'>ROUTEADV OFF</option>"
    "        <option value='CMD:DISPLAY CLEAR'>DISPLAY CLEAR</option>"
    "        <option value='CMD:DISPLAY TEST'>DISPLAY TEST</option>"
    "      </select>"
    "      <input id='cfg_cmd_text' maxlength='96' placeholder='oder eigener CMD: Text'>"
    "    </div>"
    "  </div>"

    "</div>";
}

const char *mr_web_cmdcfg_js(void)
{
    return
    "function cmdCfgApplyPreset(){"
    "  let sel = document.getElementById('cfg_cmd_text_select');"
    "  let inp = document.getElementById('cfg_cmd_text');"
    "  if(sel && inp && sel.value) inp.value = sel.value;"
    "}"

    "function cmdCfgRefresh(c){"
    "  let trig = String(c.cmd_trigger_gpio ?? '-1');"
    "  if(trig === '17' || trig === '18') trig = '-1';"
    "  document.getElementById('cfg_cmd_trigger_gpio').value = trig;"
    "  document.getElementById('cfg_cmd_dst').value = (c.cmd_dst || '');"
    "  let cmd = (c.cmd_text || '');"
    "  document.getElementById('cfg_cmd_text').value = cmd;"
    "  let sel = document.getElementById('cfg_cmd_text_select');"
    "  if(sel) sel.value = cmd;"
    "  document.getElementById('cfg_cmd_enable').checked = !!c.cmd_enable;"
    "}"

    "function cmdCfgCollectOps(){"
    "  let trig = document.getElementById('cfg_cmd_trigger_gpio').value;"
    "  if(trig === '17' || trig === '18') trig = '-1';"
    "  return ["
    "    ['cmd_enable', document.getElementById('cfg_cmd_enable').checked ? '1' : '0'],"
    "    ['cmd_trigger_gpio', trig],"
    "    ['cmd_dst', document.getElementById('cfg_cmd_dst').value],"
    "    ['cmd_text', document.getElementById('cfg_cmd_text').value]"
    "  ];"
    "}";
}

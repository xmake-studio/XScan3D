<script lang="ts">
  import { fade, fly } from "svelte/transition";
  import { B } from "../api";
  import { bytes, fixed } from "../format";
  import { t } from "../i18n.svelte";
  import {
    app, applyCalibration, calibrationScan, scanTitle, startCalibration, updateSettings,
  } from "../state.svelte";
  import type { ColorMode, Quality } from "../types";
  import Icon from "./Icon.svelte";
  import Button from "./ui/Button.svelte";
  import NumberField from "./ui/NumberField.svelte";
  import Segmented from "./ui/Segmented.svelte";
  import Slider from "./ui/Slider.svelte";
  import Toggle from "./ui/Toggle.svelte";

  const SECTIONS = [
    { id: "general", icon: "gear" },
    { id: "scanning", icon: "radar" },
    { id: "display", icon: "eye" },
    { id: "calibration", icon: "target" },
    { id: "merging", icon: "link" },
    { id: "surface", icon: "surface" },
    { id: "device", icon: "usb" },
    { id: "about", icon: "info" },
  ];

  let s = $derived(app.settings);
  let m = $derived(app.settings.mount);
  let calibScan = $derived(calibrationScan());
  let manualPort = $state("");

  $effect(() => {
    if (!manualPort && app.device.ports.length) manualPort = (app.device.ports.find((p) => p.isScanner) ?? app.device.ports[0]).device;
  });

  function close() {
    app.settingsOpen = false;
  }

  async function changeLibrary() {
    const dir = await B().pickFolder(app.library.root);
    if (dir) updateSettings({ libraryDir: dir });
  }

  const PHASE: Record<string, string> = {
    ready: "device.ready",
    connecting: "device.connecting",
    silent: "device.silent",
    error: "device.error",
    searching: "device.searching",
    "": "device.searching",
  };

  // Where the calibration stands relative to the copy in the scanner's flash.
  let calibStore = $derived(
    app.device.calib ? app.device.calib : s.device.calibPending ? "pending" : "none",
  );

  function mountPatch(p: Record<string, unknown>) {
    updateSettings({ mount: p });
  }
</script>

<svelte:window onkeydown={(e) => e.key === "Escape" && !app.confirm && close()} />

<div class="scrim" transition:fade={{ duration: 200 }} onclick={close} role="presentation" data-modal-open></div>
<div class="sheet glass-strong" role="dialog" aria-modal="true" aria-label={t("settings.title")} transition:fly={{ y: 24, duration: 320 }}>
  <nav>
    <div class="nav-title">{t("settings.title")}</div>
    {#each SECTIONS as sec (sec.id)}
      <button class="nav" class:on={app.settingsSection === sec.id} onclick={() => (app.settingsSection = sec.id)}>
        <span class="ni"><Icon name={sec.icon} size={15} stroke={2} /></span>
        {t(`settings.${sec.id}`)}
      </button>
    {/each}
  </nav>

  <section>
    <header>
      <h2>{t(`settings.${app.settingsSection}`)}</h2>
      <button class="close" onclick={close} aria-label={t("common.close")}><Icon name="x" size={16} stroke={2} /></button>
    </header>

    <div class="content">
      {#if app.settingsSection === "general"}
        <div class="card">
          <div class="row">
            <div class="lbl">{t("settings.language")}</div>
            <Segmented
              value={s.language}
              options={[
                { value: "system", label: t("settings.system") },
                { value: "ru", label: "Русский" },
                { value: "en", label: "English" },
              ]}
              onchange={(v) => updateSettings({ language: v })}
            />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.theme")}</div>
            <Segmented
              value={s.theme}
              options={[
                { value: "system", label: t("settings.system") },
                { value: "light", icon: "sun", label: t("settings.light") },
                { value: "dark", icon: "moon", label: t("settings.dark") },
              ]}
              onchange={(v) => updateSettings({ theme: v })}
            />
          </div>
        </div>
        <div class="card">
          <div class="row col">
            <div class="lbl">{t("settings.library")}</div>
            <div class="path">
              <span class="mono" title={app.library.root}>{app.library.root}</span>
              <div class="btns">
                <Button size="sm" icon="folder" onclick={() => B().openFolder(app.library.root)}>{t("settings.openFolder")}</Button>
                <Button size="sm" onclick={changeLibrary}>{t("settings.change")}</Button>
                {#if s.libraryDir}<Button size="sm" variant="ghost" onclick={() => updateSettings({ libraryDir: null })}>{t("settings.resetDefault")}</Button>{/if}
              </div>
            </div>
          </div>
        </div>
      {:else if app.settingsSection === "scanning"}
        <div class="card">
          <div class="row">
            <div class="lbl">{t("settings.startDelay")}<div class="hint">{t("settings.startDelayHint")}</div></div>
            <Segmented
              value={s.scan.startDelay}
              options={[
                { value: 0, label: t("settings.off") },
                { value: 3, label: `3 ${t("common.s")}` },
                { value: 5, label: `5 ${t("common.s")}` },
                { value: 10, label: `10 ${t("common.s")}` },
              ]}
              onchange={(v) => updateSettings({ scan: { startDelay: v } })}
            />
          </div>
        </div>
        <div class="card">
          <div class="row">
            <div class="lbl">{t("settings.mode")}<div class="hint">{t("settings.modeHint")}</div></div>
            <Segmented
              value={s.scan.stepped ? 1 : 0}
              options={[
                { value: 0, label: t("settings.continuous") },
                { value: 1, label: t("settings.stepped") },
              ]}
              onchange={(v) => updateSettings({ scan: { stepped: v === 1 } })}
            />
          </div>
          {#if s.scan.stepped}
            <div class="row" transition:fade={{ duration: 150 }}>
              <div class="lbl">{t("settings.dwell")}</div>
              <NumberField value={s.scan.dwellMs} min={50} max={5000} step={50} digits={0} unit={t("common.mm") === "мм" ? "мс" : "ms"} onchange={(v) => updateSettings({ scan: { dwellMs: v } })} />
            </div>
          {/if}
          <div class="row">
            <div class="lbl">{t("settings.sweep")}<div class="hint">{t("settings.sweepHint")}</div></div>
            <div class="inline">
              <Segmented
                value={s.scan.sweepDeg > 0 ? 1 : 0}
                options={[
                  { value: 0, label: t("settings.auto") },
                  { value: 1, label: t("settings.sweepFixed") },
                ]}
                onchange={(v) => updateSettings({ scan: { sweepDeg: v === 1 ? 90 : 0 } })}
              />
              {#if s.scan.sweepDeg > 0}
                <span class="pm" aria-hidden="true">±</span>
                <NumberField value={s.scan.sweepDeg} min={1} max={180} step={5} digits={0} unit="°" width={76} onchange={(v) => updateSettings({ scan: { sweepDeg: v } })} />
              {/if}
            </div>
          </div>
        </div>
      {:else if app.settingsSection === "display"}
        <div class="card">
          <div class="row">
            <div class="lbl">{t("settings.colorMode")}</div>
            <Segmented
              size="sm"
              value={s.view.colorMode}
              options={[
                { value: "height" as ColorMode, label: t("view.height") },
                { value: "distance" as ColorMode, label: t("view.distance") },
                { value: "mono" as ColorMode, label: t("view.mono") },
                { value: "scan" as ColorMode, label: t("view.byScan") },
              ]}
              onchange={(v) => updateSettings({ view: { colorMode: v } })}
            />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.pointSize")}</div>
            <div class="slider"><Slider value={s.view.pointSize} min={0.4} max={2.5} step={0.05} oninput={(v) => (app.settings.view.pointSize = v)} onchange={(v) => updateSettings({ view: { pointSize: v } })} /></div>
          </div>
          <div class="row">
            <div class="lbl">{t("settings.edl")}<div class="hint">{t("settings.edlHint")}</div></div>
            <Toggle checked={s.view.edl} onchange={(v) => updateSettings({ view: { edl: v } })} />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.grid")}</div>
            <Toggle checked={s.view.grid} onchange={(v) => updateSettings({ view: { grid: v } })} />
          </div>
        </div>
        <div class="card">
          <div class="row">
            <div class="lbl">{t("settings.maxRange")}<div class="hint">{t("settings.maxRangeHint")}</div></div>
            <NumberField value={m.maxRange / 1000} min={0} max={40} step={0.5} digits={1} unit="m" onchange={(v) => mountPatch({ maxRange: Math.round(v * 1000) })} />
          </div>
        </div>
      {:else if app.settingsSection === "calibration"}
        <div class="card">
          <div class="row">
            <div class="lbl">{t("settings.microstep")}<div class="hint">{t("settings.microstepHint")}</div></div>
            <Toggle checked={m.microstepAuto} onchange={(v) => mountPatch({ microstepAuto: v })} />
          </div>
          <div class="row">
            <div class="lbl">
              {t("settings.rangeCorrection")}
              <div class="hint">{t("settings.rangeHint")} {m.rangeError ? t("settings.rangeAmount", { mm: fixed(app.rangeAt3m || 0, 1) }) : t("settings.notCalibrated")}</div>
            </div>
            <Toggle checked={m.rangeCorrection && !!m.rangeError} disabled={!m.rangeError} onchange={(v) => mountPatch({ rangeCorrection: v })} />
          </div>
        </div>

        <div class="card accent">
          <div class="cal-head">
            <div class="cal-icon"><Icon name="target" size={20} /></div>
            <div class="lbl">
              {t("settings.calibrateTitle")}
              <div class="hint">{t("settings.calibrateBody")}</div>
            </div>
          </div>
          {#if app.calib.running}
            <div class="progress">
              <div class="bar"><span style="width:{Math.round(app.calib.fraction * 100)}%"></span></div>
              <div class="prow">
                <span class="muted">{app.calib.stage ? t(`settings.calibStage.${app.calib.stage}`) : t("settings.calibrating")}</span>
                <Button size="sm" variant="ghost" onclick={() => B().calibrateCancel()}>{t("settings.calibCancel")}</Button>
              </div>
            </div>
          {:else if app.calib.result}
            {@const r = app.calib.result}
            <table>
              <thead><tr><th></th><th>{t("settings.calibNow")}</th><th>{t("settings.calibFitted")}</th></tr></thead>
              <tbody>
                <tr><td>{t("settings.roll")}</td><td>{fixed(m.lidarRotation, 2)}°</td><td class="new">{fixed(r.rotation, 2)}°</td></tr>
                <tr><td>{t("settings.spacing")}</td><td>{fixed(m.emitterSpacing, 1)} {t("common.mm")}</td><td class="new">{fixed(r.spacing, 1)} {t("common.mm")}</td></tr>
                <tr><td>{t("settings.tilt")}</td><td>{fixed(m.scanTilt, 2)}°</td><td class="new">{fixed(r.tilt, 2)}°</td></tr>
                <tr><td>{t("settings.calibRange")}</td><td>{fixed(r.rangeAt3mBefore, 1)} {t("common.mm")}</td><td class="new">{fixed(r.rangeAt3mAfter, 1)} {t("common.mm")}</td></tr>
                <tr class="sub"><td colspan="3">{t("settings.calibErrors")}</td></tr>
                <tr><td>{t("settings.calibPlanes")}</td><td>{fixed(r.before.planes, 2)}</td><td class="new">{fixed(r.after.planes, 2)}</td></tr>
                <tr><td>{t("settings.calibSeamUp")}</td><td>{fixed(r.before.up, 2)}</td><td class="new">{fixed(r.after.up, 2)}</td></tr>
                <tr><td>{t("settings.calibSeamHorizon")}</td><td>{fixed(r.before.horizon, 2)}</td><td class="new">{fixed(r.after.horizon, 2)}</td></tr>
                <tr><td>{t("settings.calibSeamDown")}</td><td>{fixed(r.before.down, 2)}</td><td class="new">{fixed(r.after.down, 2)}</td></tr>
              </tbody>
            </table>
            <div class="btns end">
              <Button variant="secondary" onclick={() => (app.calib = { ...app.calib, result: null })}>{t("settings.discard")}</Button>
              <Button variant="primary" onclick={applyCalibration}>{t("settings.apply")}</Button>
            </div>
          {:else}
            <div class="prow">
              <span class="muted small">{calibScan ? scanTitle(calibScan) : t("settings.calibrateNeedScan")}</span>
              <Button variant="primary" icon="target" disabled={!calibScan} onclick={startCalibration}>{t("settings.calibrateButton")}</Button>
            </div>
          {/if}
        </div>

        <div class="card">
          <div class="card-title">{t("settings.geometry")}</div>
          <div class="store {calibStore}">
            <Icon name="usb" size={13} stroke={2} />
            <span>{t(`settings.calibStore.${calibStore}`)}</span>
          </div>
          <div class="row">
            <div class="lbl">{t("settings.roll")}</div>
            <NumberField value={m.lidarRotation} min={-180} max={180} step={0.05} digits={2} unit="°" onchange={(v) => mountPatch({ lidarRotation: v })} />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.spacing")}</div>
            <NumberField value={m.emitterSpacing} min={-200} max={200} step={0.5} digits={1} unit={t("common.mm")} onchange={(v) => mountPatch({ emitterSpacing: v })} />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.tilt")}</div>
            <NumberField value={m.scanTilt} min={-10} max={10} step={0.05} digits={2} unit="°" onchange={(v) => mountPatch({ scanTilt: v })} />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.half")}</div>
            <Segmented
              size="sm"
              value={m.scanHalf}
              options={[
                { value: "both", label: t("settings.halfBoth") },
                { value: "a", label: t("settings.halfA") },
                { value: "b", label: t("settings.halfB") },
              ]}
              onchange={(v) => mountPatch({ scanHalf: v })}
            />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.reverse")}</div>
            <Toggle checked={m.lidarReverse} onchange={(v) => mountPatch({ lidarReverse: v })} />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.flip")}</div>
            <Toggle checked={m.flipUpright} onchange={(v) => mountPatch({ flipUpright: v })} />
          </div>
        </div>
      {:else if app.settingsSection === "merging"}
        <div class="card">
          <div class="row">
            <div class="lbl">{t("settings.moveMode")}<div class="hint">{t("settings.moveModeHint")}</div></div>
            <Segmented
              size="sm"
              value={s.merge.mode}
              options={[
                { value: "auto", label: t("settings.moveAuto") },
                { value: "small", label: t("settings.moveSmall") },
                { value: "hint", label: t("settings.moveHint") },
              ]}
              onchange={(v) => updateSettings({ merge: { mode: v } })}
            />
          </div>
          {#if s.merge.mode === "hint"}
            <div class="row" transition:fade={{ duration: 150 }}>
              <div class="lbl">{t("settings.yaw")}</div>
              <NumberField value={s.merge.yawHint} min={-180} max={180} step={5} digits={0} unit="°" onchange={(v) => updateSettings({ merge: { yawHint: v } })} />
            </div>
          {/if}
          <div class="row">
            <div class="lbl">{t("settings.detail")}</div>
            <NumberField value={s.merge.voxel} min={5} max={500} step={5} digits={0} unit={t("common.mm")} onchange={(v) => updateSettings({ merge: { voxel: v } })} />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.foliage")}<div class="hint">{t("settings.foliageHint")}</div></div>
            <Toggle checked={s.merge.ignoreFoliage} onchange={(v) => updateSettings({ merge: { ignoreFoliage: v } })} />
          </div>
        </div>
      {:else if app.settingsSection === "surface"}
        <div class="card">
          <div class="row">
            <div class="lbl">{t("settings.meshEnable")}<div class="hint">{t("settings.meshEnableHint")}</div></div>
            <Toggle checked={s.mesh.enabled} onchange={(v) => updateSettings({ mesh: { enabled: v } })} />
          </div>
        </div>
        {#if s.mesh.enabled}
          <div class="card" transition:fade={{ duration: 160 }}>
            <div class="row">
              <div class="lbl">{t("settings.quality")}<div class="hint">{t("settings.qualityHint")}</div></div>
              <Segmented
                value={s.mesh.quality}
                options={[
                  { value: "medium" as Quality, label: t("settings.qMedium") },
                  { value: "high" as Quality, label: t("settings.qHigh") },
                  { value: "max" as Quality, label: t("settings.qMax") },
                ]}
                onchange={(v) => updateSettings({ mesh: { quality: v } })}
              />
            </div>
            <div class="row">
              <div class="lbl">{t("settings.trim")}<div class="hint">{t("settings.trimHint")}</div></div>
              <NumberField value={s.mesh.trim} min={0} max={20} step={0.5} digits={1} unit="×" onchange={(v) => updateSettings({ mesh: { trim: v } })} />
            </div>
            <div class="row">
              <div class="lbl">{t("settings.budget")}</div>
              <NumberField value={s.mesh.budget / 1e6} min={0.1} max={20} step={0.5} digits={1} unit={t("units.m")} onchange={(v) => updateSettings({ mesh: { budget: Math.round(v * 1e6) } })} />
            </div>
            <div class="row">
              <div class="lbl">{t("settings.smooth")}</div>
              <NumberField value={s.mesh.smooth} min={0} max={20} step={1} digits={0} onchange={(v) => updateSettings({ mesh: { smooth: v } })} />
            </div>
          </div>
        {/if}
      {:else if app.settingsSection === "device"}
        <div class="card">
          <div class="card-title">{t("settings.status")}</div>
          <div class="status">
            <span class="sdot {app.device.phase}"></span>
            <div>
              <div class="strong">{t(PHASE[app.device.phase] ?? "device.searching")}</div>
              <div class="muted small tnum">
                {#if app.device.port}{t("settings.port")}: {app.device.port} · {/if}{t("settings.received", { n: bytes(app.device.bytesIn) })}
                {#if app.device.config} · ±{fixed(app.device.config.degrees, 0)}°{/if}
              </div>
            </div>
          </div>
        </div>
        <div class="card">
          <div class="row">
            <div class="lbl">{t("settings.autoConnect")}<div class="hint">{t("settings.autoConnectHint")}</div></div>
            <Toggle checked={s.device.autoConnect} onchange={(v) => updateSettings({ device: { autoConnect: v } })} />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.dtr")}<div class="hint">{t("settings.dtrHint")}</div></div>
            <Toggle checked={s.device.dtr} onchange={(v) => updateSettings({ device: { dtr: v } })} />
          </div>
          <div class="row">
            <div class="lbl">{t("settings.manual")}</div>
            <div class="inline">
              {#if app.device.ports.length}
                <select bind:value={manualPort}>
                  {#each app.device.ports as p (p.device)}
                    <option value={p.device}>{p.device}{p.product ? ` — ${p.product}` : ""}</option>
                  {/each}
                </select>
              {:else}
                <span class="muted small">{t("settings.noPorts")}</span>
              {/if}
              {#if app.device.port}
                <Button size="sm" onclick={() => B().deviceDisconnect()}>{t("settings.disconnect")}</Button>
              {:else}
                <Button size="sm" variant="tinted" disabled={!manualPort} onclick={() => B().deviceConnect(manualPort)}>{t("settings.connect")}</Button>
              {/if}
            </div>
          </div>
        </div>
        <div class="card">
          <div class="card-title">{t("settings.maintenance")}</div>
          <div class="btns wrap">
            <Button size="sm" icon="home" disabled={!app.connected || app.scanning} onclick={() => B().deviceCommand("home")}>{t("settings.home")}</Button>
            <Button size="sm" icon="undo" disabled={!app.connected || app.scanning} onclick={() => B().deviceCommand("unwrap")}>{t("settings.unwrap")}</Button>
            <Button size="sm" icon="bell" disabled={!app.connected || app.scanning} onclick={() => B().deviceCommand("beep")}>{t("settings.beep")}</Button>
          </div>
        </div>
        <div class="card">
          <div class="row">
            <div class="card-title">{t("settings.log")}</div>
            <Button size="sm" variant="ghost" icon="folder" onclick={() => B().openFolder(app.logDir)}>{t("settings.openLogs")}</Button>
          </div>
          <pre class="log">{app.deviceLog.slice(-60).join("\n") || "—"}</pre>
        </div>
      {:else}
        <div class="about">
          <svg width="72" height="72" viewBox="0 0 1024 1024" aria-hidden="true">
            <defs>
              <linearGradient id="lg2" x1="0" y1="0" x2="1" y2="1"><stop offset="0" stop-color="#2F8CFF" /><stop offset="1" stop-color="#5B3CE0" /></linearGradient>
            </defs>
            <rect x="48" y="48" width="928" height="928" rx="216" fill="url(#lg2)" />
            <g fill="none" stroke="#fff" stroke-linecap="round">
              <circle cx="512" cy="512" r="262" stroke-width="30" />
              <ellipse cx="512" cy="512" rx="262" ry="96" stroke-width="26" stroke-opacity="0.78" stroke-dasharray="0.1 46" />
              <ellipse cx="512" cy="512" rx="100" ry="262" stroke-width="26" stroke-opacity="0.78" stroke-dasharray="0.1 46" />
            </g>
            <circle cx="512" cy="512" r="40" fill="#fff" />
          </svg>
          <h3>XScan3D</h3>
          <div class="muted">{t("settings.version", { v: app.version })}</div>
          <p>{t("settings.aboutBody")}</p>
          <p class="faint small">{t("settings.credits")}</p>
        </div>
      {/if}
    </div>
  </section>
</div>

<style>
  .scrim {
    position: fixed;
    inset: 0;
    background: var(--scrim);
    z-index: 80;
  }
  .sheet {
    position: fixed;
    z-index: 81;
    left: 50%;
    top: 50%;
    translate: -50% -50%;
    width: min(860px, calc(100vw - 48px));
    height: min(620px, calc(100vh - 80px));
    border-radius: 22px;
    display: flex;
    overflow: hidden;
  }
  nav {
    width: 210px;
    flex: none;
    padding: 16px 10px;
    border-right: 1px solid var(--sep);
    display: flex;
    flex-direction: column;
    gap: 2px;
    background: color-mix(in srgb, var(--fill) 50%, transparent);
  }
  .nav-title {
    font-size: 11px;
    font-weight: 650;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: var(--text-3);
    padding: 4px 10px 10px;
  }
  .nav {
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 6px 8px;
    border-radius: 8px;
    font-weight: 560;
    text-align: left;
    transition: background 0.15s;
  }
  .nav:hover {
    background: var(--fill);
  }
  .nav.on {
    background: var(--accent);
    color: #fff;
  }
  .ni {
    width: 24px;
    height: 24px;
    border-radius: 7px;
    display: grid;
    place-items: center;
    background: var(--fill-2);
    color: var(--text);
  }
  .nav.on .ni {
    background: rgba(255, 255, 255, 0.22);
    color: #fff;
  }
  section {
    flex: 1;
    min-width: 0;
    display: flex;
    flex-direction: column;
  }
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 16px 16px 8px 24px;
  }
  h2 {
    margin: 0;
    font-size: 20px;
    font-weight: 700;
    letter-spacing: -0.02em;
  }
  .close {
    width: 28px;
    height: 28px;
    border-radius: 50%;
    display: grid;
    place-items: center;
    background: var(--fill);
    color: var(--text-2);
  }
  .close:hover {
    background: var(--fill-2);
    color: var(--text);
  }
  .content {
    flex: 1;
    overflow-y: auto;
    padding: 6px 24px 24px;
    display: flex;
    flex-direction: column;
    gap: 14px;
  }
  .card {
    border-radius: 14px;
    background: var(--card);
    box-shadow: inset 0 0 0 0.5px var(--glass-border);
    padding: 4px 14px;
  }
  .card.accent {
    padding: 14px;
    background: linear-gradient(135deg, var(--accent-soft), transparent 70%), var(--card);
  }
  .card-title {
    font-weight: 650;
    padding: 10px 0 4px;
  }
  .store {
    display: flex;
    align-items: center;
    gap: 6px;
    padding: 0 0 6px;
    font-size: 12px;
    color: var(--text-2);
  }
  .store.synced {
    color: var(--green);
  }
  .store.failed {
    color: var(--red);
  }
  .store.writing,
  .store.reading,
  .store.pending {
    color: var(--orange);
  }
  .row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 16px;
    min-height: 46px;
    padding: 8px 0;
  }
  .row + .row {
    border-top: 1px solid var(--sep);
  }
  .row.col {
    flex-direction: column;
    align-items: stretch;
  }
  .lbl {
    font-weight: 560;
    min-width: 0;
  }
  .hint {
    font-weight: 400;
    font-size: 12px;
    color: var(--text-2);
    margin-top: 2px;
    max-width: 380px;
  }
  .inline {
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .pm {
    margin-right: -4px;
    color: var(--text-3);
  }
  .slider {
    width: 220px;
  }
  .path {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .mono {
    font-family: ui-monospace, "Cascadia Mono", Consolas, monospace;
    font-size: 12px;
    color: var(--text-2);
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .btns {
    display: flex;
    gap: 6px;
  }
  .btns.end {
    justify-content: flex-end;
    margin-top: 10px;
  }
  .btns.wrap {
    flex-wrap: wrap;
    padding: 4px 0 12px;
  }
  .cal-head {
    display: flex;
    gap: 12px;
    align-items: flex-start;
    margin-bottom: 12px;
  }
  .cal-icon {
    width: 36px;
    height: 36px;
    flex: none;
    border-radius: 10px;
    display: grid;
    place-items: center;
    background: var(--accent);
    color: #fff;
  }
  .progress .bar {
    height: 6px;
    border-radius: 999px;
    background: var(--fill-2);
    overflow: hidden;
  }
  .progress .bar span {
    display: block;
    height: 100%;
    background: var(--accent);
    border-radius: 999px;
    transition: width 0.4s var(--ease);
  }
  .prow {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 12px;
    margin-top: 8px;
  }
  .small {
    font-size: 12px;
  }
  .strong {
    font-weight: 600;
  }
  table {
    width: 100%;
    border-collapse: collapse;
    font-variant-numeric: tabular-nums;
  }
  th {
    text-align: right;
    font-size: 11px;
    color: var(--text-2);
    font-weight: 600;
    padding: 4px 6px;
  }
  td {
    padding: 4px 6px;
    text-align: right;
  }
  td:first-child {
    text-align: left;
    color: var(--text-2);
  }
  td.new {
    font-weight: 650;
  }
  tr.sub td {
    padding-top: 10px;
    font-size: 11px;
    font-style: italic;
    color: var(--text-3);
  }
  .status {
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 6px 0 12px;
  }
  .sdot {
    width: 12px;
    height: 12px;
    border-radius: 50%;
    background: var(--orange);
    animation: pulse-soft 1.6s ease-in-out infinite;
  }
  .sdot.ready {
    background: var(--green);
    animation: none;
    box-shadow: 0 0 10px var(--green);
  }
  .sdot.error {
    background: var(--red);
    animation: none;
  }
  select {
    height: 28px;
    border-radius: 7px;
    border: 0;
    padding: 0 8px;
    background: var(--field);
    box-shadow: inset 0 0 0 0.5px var(--sep);
    max-width: 240px;
  }
  select option {
    background: var(--glass-strong);
    color: var(--text);
  }
  .log {
    margin: 0 0 12px;
    max-height: 170px;
    overflow: auto;
    padding: 10px;
    border-radius: 10px;
    background: var(--field);
    font: 11px/1.45 ui-monospace, "Cascadia Mono", Consolas, monospace;
    color: var(--text-2);
    user-select: text;
    white-space: pre-wrap;
  }
  .about {
    text-align: center;
    padding: 30px 40px;
  }
  .about h3 {
    margin: 14px 0 2px;
    font-size: 22px;
    font-weight: 700;
  }
  .about p {
    margin: 16px auto 0;
    max-width: 420px;
    color: var(--text-2);
  }
</style>

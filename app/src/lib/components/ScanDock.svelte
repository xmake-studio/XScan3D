<script lang="ts">
  import { slide } from "svelte/transition";
  import { clock, compact, duration as fmtDuration } from "../format";
  import { t } from "../i18n.svelte";
  import { app, cancelCountdown, startScan, stopScan, updateSettings } from "../state.svelte";
  import ScanButton from "./ScanButton.svelte";
  import Slider from "./ui/Slider.svelte";

  // The duration slider is exponential: equal slider travel is an equal
  // ratio of time, so 15 s and 10 min both get usable resolution. Ten
  // minutes is as long as a sweep is worth: past that the platform has
  // already crossed every azimuth at the lidar's own resolution.
  const T_MIN = 15;
  const T_MAX = 600;
  const toT = (p: number) => T_MIN * Math.pow(T_MAX / T_MIN, p);
  const toP = (s: number) => Math.log(Math.min(T_MAX, Math.max(T_MIN, s)) / T_MIN) / Math.log(T_MAX / T_MIN);
  function round(s: number) {
    if (s < 60) return Math.round(s / 5) * 5;
    if (s < 300) return Math.round(s / 15) * 15;
    return Math.round(s / 30) * 30;
  }

  let pos = $state(0);
  let dragging = $state(false);
  $effect(() => {
    if (!dragging && app.settings) pos = toP(app.settings.scan.duration);
  });
  let seconds = $derived(round(toT(pos)));

  // Frames reach ~375/s; after dropouts and the near-range cut about 2300
  // points a second survive (measured on scans/room.bin).
  let estimate = $derived(seconds * 2300);
  let quality = $derived(seconds < 45 ? t("scan.quick") : seconds < 150 ? t("scan.standard") : seconds < 420 ? t("scan.detailed") : t("scan.maximum"));

  let mode = $derived.by(() => {
    if (app.countdown > 0) return "countdown" as const;
    if (app.scan) return app.scan.phase === "scanning" ? ("scanning" as const) : ("busy" as const);
    if (!app.connected) return "off" as const;
    return "ready" as const;
  });

  let title = $derived.by(() => {
    if (app.countdown > 0) return t("scan.countdown", { n: app.countdown });
    const s = app.scan;
    if (s) {
      if (s.phase === "starting") return t("scan.starting");
      if (s.phase === "preparing") return t("scan.preparing");
      if (s.phase === "finishing") return t("scan.finishing");
      return `${t("scan.scanning")} · ${Math.round(s.progress * 100)}%`;
    }
    if (app.device.phase === "silent") return t("device.silent");
    if (!app.connected) return t("scan.connectFirst");
    return t("scan.start");
  });

  let subtitle = $derived.by(() => {
    if (app.countdown > 0) return t("scan.tapToCancel");
    const s = app.scan;
    if (s) {
      if (s.phase === "preparing" || s.phase === "starting") return t("scan.preparingHint");
      if (s.phase === "finishing") return t("scan.finishingHint");
      return `${t("scan.remaining", { time: clock(s.remaining) })} · ${t("scan.points", { n: compact(s.points) })}`;
    }
    if (app.device.phase === "silent") return t("scan.silentHint");
    if (!app.connected) return t("scan.searchingHint");
    return `${fmtDuration(seconds)} · ${t("scan.estimate", { points: compact(estimate) })}`;
  });

  function press() {
    if (mode === "countdown") cancelCountdown();
    else if (mode === "scanning" || mode === "busy") {
      if (app.scan?.phase !== "finishing") void stopScan();
    } else if (mode === "off") {
      app.settingsSection = "device";
      app.settingsOpen = true;
    } else void startScan();
  }

  function commit() {
    dragging = false;
    if (seconds !== app.settings.scan.duration) updateSettings({ scan: { duration: seconds } });
  }

  let showSlider = $derived(!app.scan && app.countdown === 0);
</script>

<div class="dock glass" class:live={mode === "scanning" || mode === "busy"}>
  <ScanButton {mode} progress={app.scan?.progress ?? 0} count={app.countdown} onclick={press} label={title} />
  <div class="body">
    <div class="head">
      <div class="title">{title}</div>
      {#if showSlider && app.connected}
        <span class="chip">{quality}{app.settings.scan.stepped ? ` · ${t("scan.stepped")}` : ""}</span>
      {/if}
    </div>
    <div class="sub tnum">{subtitle}</div>
    {#if showSlider}
      <div class="slider" transition:slide={{ duration: 260 }}>
        <Slider
          bind:value={pos}
          min={0}
          max={1}
          step={0.001}
          label={t("scan.duration")}
          oninput={() => (dragging = true)}
          onchange={commit}
        />
        <div class="ticks faint">
          <span>15 {t("units.sec")}</span>
          <span>30 {t("units.sec")}</span>
          <span>1 {t("units.min")}</span>
          <span>3 {t("units.min")}</span>
          <span>10 {t("units.min")}</span>
        </div>
      </div>
    {/if}
  </div>
</div>

<style>
  .dock {
    display: flex;
    align-items: center;
    gap: 18px;
    width: 470px;
    padding: 14px 22px 14px 14px;
    border-radius: 30px;
    transition: width 0.4s var(--ease), padding 0.3s var(--ease);
  }
  .dock.live {
    width: 400px;
  }
  .body {
    flex: 1;
    min-width: 0;
  }
  .head {
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .title {
    font-size: 17px;
    font-weight: 650;
    letter-spacing: -0.015em;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .chip {
    margin-left: auto;
    padding: 2px 8px;
    border-radius: 999px;
    background: var(--accent-soft);
    color: var(--accent);
    font-size: 11px;
    font-weight: 600;
    white-space: nowrap;
  }
  .sub {
    margin-top: 1px;
    color: var(--text-2);
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .slider {
    margin-top: 8px;
  }
  /* Tick labels sit at their exponential positions: 15 s .. 10 min. */
  .ticks {
    position: relative;
    height: 13px;
    font-size: 10px;
    margin: 1px 10px 0;
  }
  .ticks span {
    position: absolute;
    transform: translateX(-50%);
    white-space: nowrap;
  }
  .ticks span:nth-child(1) {
    left: 0%;
  }
  .ticks span:nth-child(2) {
    left: 18.8%;
  }
  .ticks span:nth-child(3) {
    left: 37.6%;
  }
  .ticks span:nth-child(4) {
    left: 67.3%;
  }
  .ticks span:nth-child(5) {
    left: 100%;
  }
</style>

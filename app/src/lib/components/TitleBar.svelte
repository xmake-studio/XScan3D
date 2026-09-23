<script lang="ts">
  import { fade } from "svelte/transition";
  import { B } from "../api";
  import { t } from "../i18n.svelte";
  import { app, describeSelection } from "../state.svelte";
  import { tip } from "../tooltip";
  import Icon from "./Icon.svelte";

  let maximized = $state(false);

  $effect(() => {
    let off: (() => void) | undefined;
    B().onWindowResize(async () => (maximized = await B().windowIsMaximized())).then((u) => (off = u));
    B().windowIsMaximized().then((m) => (maximized = m));
    return () => off?.();
  });

  let status = $derived.by(() => {
    if (app.scanning) return { tone: "live", text: t("device.scanning") };
    switch (app.device.phase) {
      case "ready":
        return { tone: "ok", text: t("device.ready") };
      case "connecting":
        return { tone: "wait", text: t("device.connecting") };
      case "silent":
        return { tone: "warn", text: t("device.silent") };
      case "error":
        return { tone: "bad", text: t("device.error") };
      default:
        return { tone: "wait", text: t("device.searching") };
    }
  });

  let title = $derived(app.selection.length ? describeSelection() : "");

  function openDevice() {
    app.settingsSection = "device";
    app.settingsOpen = true;
  }
</script>

<header class="bar" data-tauri-drag-region>
  <div class="left">
    <button class="icon-btn" onclick={() => (app.sidebarOpen = !app.sidebarOpen)} use:tip={t("view.library")} aria-label={t("view.library")}>
      <Icon name="sidebar" size={18} />
    </button>
    <div class="brand" data-tauri-drag-region>
      <svg width="18" height="18" viewBox="0 0 1024 1024" aria-hidden="true">
        <defs>
          <linearGradient id="lg" x1="0" y1="0" x2="1" y2="1">
            <stop offset="0" stop-color="#2F8CFF" />
            <stop offset="1" stop-color="#5B3CE0" />
          </linearGradient>
        </defs>
        <rect x="48" y="48" width="928" height="928" rx="216" fill="url(#lg)" />
        <circle cx="512" cy="512" r="262" fill="none" stroke="#fff" stroke-width="64" />
        <circle cx="512" cy="512" r="64" fill="#fff" />
      </svg>
      <span data-tauri-drag-region>XScan3D</span>
    </div>
  </div>

  <div class="center" data-tauri-drag-region>
    {#key title}
      <span class="title" data-tauri-drag-region in:fade={{ duration: 180 }}>{title}</span>
    {/key}
  </div>

  <div class="right">
    <button class="pill {status.tone}" onclick={openDevice}>
      <span class="dot"></span>
      <span>{status.text}</span>
    </button>
    <button class="icon-btn" onclick={() => (app.settingsOpen = true)} use:tip={t("view.settings")} aria-label={t("view.settings")}>
      <Icon name="gear" size={18} />
    </button>
    <div class="win">
      <button onclick={() => B().windowMinimize()} aria-label={t("window.minimize")}><Icon name="winMin" size={16} stroke={1.2} /></button>
      <button onclick={() => B().windowToggleMaximize()} aria-label={maximized ? t("window.restore") : t("window.maximize")}>
        <Icon name={maximized ? "winRestore" : "winMax"} size={16} stroke={1.2} />
      </button>
      <button class="close" onclick={() => B().windowClose()} aria-label={t("window.close")}><Icon name="winClose" size={16} stroke={1.2} /></button>
    </div>
  </div>
</header>

<style>
  .bar {
    position: absolute;
    inset: 0 0 auto 0;
    height: var(--titlebar);
    display: grid;
    grid-template-columns: 1fr auto 1fr;
    align-items: center;
    z-index: 40;
    padding-left: 10px;
  }
  .left,
  .right {
    display: flex;
    align-items: center;
    gap: 6px;
  }
  .right {
    justify-content: flex-end;
    gap: 8px;
  }
  .brand {
    display: flex;
    align-items: center;
    gap: 8px;
    margin-left: 4px;
    font-weight: 650;
    letter-spacing: -0.01em;
    font-size: 13px;
  }
  .center {
    min-width: 0;
    max-width: 44vw;
    text-align: center;
  }
  .title {
    display: block;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
    color: var(--text-2);
    font-weight: 560;
  }
  .icon-btn {
    width: 32px;
    height: 30px;
    display: grid;
    place-items: center;
    border-radius: 8px;
    color: var(--text-2);
    transition: background 0.15s, color 0.15s;
  }
  .icon-btn:hover {
    background: var(--fill);
    color: var(--text);
  }
  .pill {
    display: flex;
    align-items: center;
    gap: 7px;
    height: 26px;
    padding: 0 11px 0 9px;
    border-radius: 999px;
    background: var(--fill);
    font-size: 12px;
    font-weight: 560;
    color: var(--text-2);
    transition: background 0.2s;
  }
  .pill:hover {
    background: var(--fill-2);
    color: var(--text);
  }
  .dot {
    width: 7px;
    height: 7px;
    border-radius: 50%;
    background: var(--text-3);
    box-shadow: 0 0 0 3px color-mix(in srgb, currentColor 0%, transparent);
  }
  .ok .dot {
    background: var(--green);
    box-shadow: 0 0 8px color-mix(in srgb, var(--green) 70%, transparent);
  }
  .live .dot {
    background: var(--red);
    animation: pulse-soft 1.2s ease-in-out infinite;
  }
  .wait .dot {
    background: var(--orange);
    animation: pulse-soft 1.6s ease-in-out infinite;
  }
  .warn .dot {
    background: var(--orange);
  }
  .bad .dot {
    background: var(--red);
  }
  .win {
    display: flex;
    height: var(--titlebar);
    margin-left: 4px;
  }
  .win button {
    width: 46px;
    height: 100%;
    display: grid;
    place-items: center;
    color: var(--text-2);
    transition: background 0.12s, color 0.12s;
  }
  .win button:hover {
    background: var(--fill);
    color: var(--text);
  }
  .win .close:hover {
    background: #e81123;
    color: #fff;
  }
</style>

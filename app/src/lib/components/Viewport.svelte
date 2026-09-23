<script lang="ts">
  import { onMount } from "svelte";
  import { fade, scale } from "svelte/transition";
  import { fixed } from "../format";
  import { t } from "../i18n.svelte";
  import { app, openFiles, updateSettings } from "../state.svelte";
  import { animatePlane, applyVisible, attach, detach, effectiveClip, onScan, showScans, syncMesh, view } from "../viewer";
  import Icon from "./Icon.svelte";
  import Button from "./ui/Button.svelte";

  let canvas: HTMLCanvasElement | undefined = $state();
  let speedToast = $state<string | null>(null);
  let flyHint = $state(false);
  let failed = $state(false);
  let speedTimer = 0;

  onMount(() => {
    try {
      const r = attach(
        canvas!,
        (v) => {
          speedToast = t("view.flySpeed", { v: fixed(v / 1000, 2) });
          window.clearTimeout(speedTimer);
          speedTimer = window.setTimeout(() => (speedToast = null), 1100);
          if (app.settings) app.settings.view.flySpeed = v;
        },
        () => {
          if (flyHint) {
            flyHint = false;
            if (!app.settings.flyHintSeen) updateSettings({ flyHintSeen: true });
          }
        },
      );
      r.onFrame = animatePlane;
    } catch {
      failed = true;
    }
    return () => detach();
  });

  // Appearance.
  $effect(() => {
    const r = view.r;
    if (!r || !app.settings) return;
    r.theme = app.theme;
    r.colorMode = app.settings.view.colorMode;
    r.pointSize = app.settings.view.pointSize;
    r.edl = app.settings.view.edl;
    r.grid = app.settings.view.grid;
    r.camera.flySpeed = app.settings.view.flySpeed;
    r.requestRender();
  });

  // Camera mode, with a one-time hint about the keys.
  $effect(() => {
    const r = view.r;
    if (!r || !app.settings) return;
    const mode = app.settings.view.camera;
    r.setCamMode(mode);
    if (mode === "fly") {
      flyHint = true;
      r.canvas.focus();
      setTimeout(() => (flyHint = false), app.settings.flyHintSeen ? 2600 : 7000);
    } else flyHint = false;
  });

  // What is on screen follows the selection.
  $effect(() => {
    const scans = app.visibleScans;
    void app.geometryVersion;
    if (!view.r || app.scan) return;
    void showScans(scans);
  });

  // The alignment preview: move and tint the scans being reviewed.
  $effect(() => {
    void app.merge?.pending;
    if (!view.r || app.scan) return;
    applyVisible(app.visibleScans);
  });

  // A sweep in progress.
  $effect(() => {
    onScan(app.scan);
  });

  $effect(() => {
    void app.mesh.show;
    void app.mesh.key;
    void app.mesh.wantKey;
    void app.scan;
    void syncMesh();
  });

  // Cut from the top: automatic below a detected ceiling (dollhouse view),
  // off, or at a height the user set.
  $effect(() => {
    const r = view.r;
    if (!r) return;
    void app.clipMode;
    void app.clipZ;
    void app.sceneTick;
    r.clipTop = effectiveClip();
    r.requestRender();
  });

  let empty = $derived(app.ready && !app.items.length && !app.scan);
  let nothingSelected = $derived(app.ready && app.items.length > 0 && !app.selection.length && !app.scan);
</script>

<div class="viewport">
  <canvas bind:this={canvas} tabindex="-1"></canvas>

  {#if failed}
    <div class="center"><div class="hero"><Icon name="alert" size={30} /><p>WebGL2 is not available.</p></div></div>
  {/if}

  {#if empty}
    <div class="center" transition:fade={{ duration: 400 }}>
      <div class="hero">
        <div class="orb">
          <span class="ring r1"></span><span class="ring r2"></span><span class="ring r3"></span><span class="core"></span>
        </div>
        <h1>{t("library.empty")}</h1>
        <p>{t("library.emptyHint")}</p>
        <div class="cta"><Button variant="secondary" icon="folder" onclick={openFiles}>{t("library.open")}</Button></div>
      </div>
    </div>
  {:else if nothingSelected}
    <div class="center" transition:fade={{ duration: 300 }}>
      <p class="hint">{t("library.title")} ←</p>
    </div>
  {/if}

  {#if app.loading.length}
    <div class="loading glass" transition:scale={{ start: 0.9, duration: 200 }}>
      <span class="spinner spin"></span>{t("library.loading")}
    </div>
  {/if}

  {#if flyHint}
    <div class="toast glass" transition:fade={{ duration: 250 }}>
      <Icon name="fly" size={15} />
      {t("view.flyToast")}
    </div>
  {:else if speedToast}
    <div class="toast glass tnum" transition:fade={{ duration: 150 }}>{speedToast}</div>
  {/if}
</div>

<style>
  .viewport {
    position: absolute;
    inset: 0;
    z-index: 0;
  }
  canvas {
    width: 100%;
    height: 100%;
    display: block;
    outline: none;
    touch-action: none;
  }
  .center {
    position: absolute;
    inset: 0;
    display: grid;
    place-items: center;
    pointer-events: none;
    padding-bottom: 120px;
  }
  .hero {
    text-align: center;
    max-width: 420px;
    color: var(--text-2);
  }
  .hero h1 {
    margin: 26px 0 8px;
    font-size: 24px;
    font-weight: 700;
    letter-spacing: -0.025em;
    color: var(--text);
  }
  .hero p {
    margin: 0;
    font-size: 14px;
  }
  .cta {
    margin-top: 18px;
    pointer-events: auto;
  }
  .hint {
    color: var(--text-3);
    font-size: 15px;
    font-weight: 560;
  }
  .orb {
    position: relative;
    width: 120px;
    height: 120px;
    margin: 0 auto;
  }
  .ring {
    position: absolute;
    inset: 0;
    border-radius: 50%;
    border: 2px dotted color-mix(in srgb, var(--accent) 70%, transparent);
    animation: orbit 9s linear infinite;
  }
  .r2 {
    inset: 14px 0;
    border-radius: 50%;
    transform: rotateX(70deg);
    animation-duration: 6s;
    animation-direction: reverse;
  }
  .r3 {
    inset: 0 14px;
    animation-duration: 12s;
  }
  .core {
    position: absolute;
    left: 50%;
    top: 50%;
    width: 16px;
    height: 16px;
    margin: -8px;
    border-radius: 50%;
    background: var(--accent);
    box-shadow: 0 0 24px var(--accent);
    animation: pulse-soft 2.4s ease-in-out infinite;
  }
  @keyframes orbit {
    to {
      transform: rotate(360deg);
    }
  }
  .loading {
    position: absolute;
    top: calc(var(--titlebar) + 10px);
    left: 50%;
    translate: -50% 0;
    display: flex;
    align-items: center;
    gap: 9px;
    padding: 7px 14px;
    border-radius: 999px;
    font-weight: 560;
    z-index: 15;
  }
  .spinner {
    width: 14px;
    height: 14px;
    border-radius: 50%;
    border: 2px solid var(--fill-3);
    border-top-color: var(--accent);
  }
  .toast {
    position: absolute;
    left: 50%;
    bottom: 150px;
    translate: -50% 0;
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 8px 14px;
    border-radius: 999px;
    font-weight: 560;
    pointer-events: none;
    z-index: 15;
  }
</style>

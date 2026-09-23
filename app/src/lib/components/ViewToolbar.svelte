<script lang="ts">
  import { scale } from "svelte/transition";
  import { t } from "../i18n.svelte";
  import { rampCss, scanColorCss } from "../render/palette";
  import { app, refreshMeshKey, updateSettings } from "../state.svelte";
  import { tip } from "../tooltip";
  import type { CamMode, ColorMode } from "../types";
  import { effectiveClip, view } from "../viewer";
  import Icon from "./Icon.svelte";
  import MeshCard from "./MeshCard.svelte";
  import Slider from "./ui/Slider.svelte";
  import Toggle from "./ui/Toggle.svelte";

  let popover = $state(false);
  let popEl: HTMLDivElement | undefined = $state();
  let btnEl: HTMLButtonElement | undefined = $state();

  function setCam(m: CamMode) {
    if (app.settings.view.camera === m) return;
    updateSettings({ view: { camera: m } });
  }

  const COLORS: { value: ColorMode; key: string }[] = [
    { value: "height", key: "view.height" },
    { value: "distance", key: "view.distance" },
    { value: "mono", key: "view.mono" },
    { value: "scan", key: "view.byScan" },
  ];

  $effect(() => {
    if (!popover) return;
    const away = (e: PointerEvent) => {
      if (popEl?.contains(e.target as Node) || btnEl?.contains(e.target as Node)) return;
      popover = false;
    };
    const key = (e: KeyboardEvent) => e.key === "Escape" && (popover = false);
    window.addEventListener("pointerdown", away, true);
    window.addEventListener("keydown", key);
    return () => {
      window.removeEventListener("pointerdown", away, true);
      window.removeEventListener("keydown", key);
    };
  });

  let clipZ = $derived.by(() => {
    void app.sceneTick;
    void app.clipMode;
    void app.clipZ;
    return effectiveClip();
  });
  let clipOn = $derived(Number.isFinite(clipZ));
  let range = $derived.by(() => {
    void app.sceneTick;
    const st = view.r?.stats ?? { zlo: -1500, zhi: 1500 };
    return { lo: st.zlo, hi: st.zhi + 400 };
  });

  function setClip(on: boolean) {
    if (on) {
      app.clipZ = null;
      app.clipMode = "on";
    } else app.clipMode = "off";
  }

  function toggleSurface() {
    app.mesh.show = !app.mesh.show;
    if (app.mesh.show) void refreshMeshKey();
  }
</script>

<div class="tools" transition:scale={{ start: 0.94, duration: 220 }}>
  <div class="group glass">
    <button class="tool" class:on={app.settings.view.camera === "orbit"} onclick={() => setCam("orbit")} use:tip={`${t("view.orbit")}\n${t("view.orbitHint")}`} aria-label={t("view.orbit")}>
      <Icon name="orbit" size={20} />
    </button>
    <button class="tool" class:on={app.settings.view.camera === "fly"} onclick={() => setCam("fly")} use:tip={`${t("view.fly")}\n${t("view.flyHint")}`} aria-label={t("view.fly")}>
      <Icon name="fly" size={19} />
    </button>
  </div>

  <div class="group glass">
    <button class="tool" onclick={() => view.r?.frame(700)} use:tip={t("view.fit")} aria-label={t("view.fit")}>
      <Icon name="frame" size={19} />
    </button>
    <button class="tool" class:on={popover} bind:this={btnEl} onclick={() => (popover = !popover)} use:tip={popover ? null : t("view.colors")} aria-label={t("view.colors")}>
      <Icon name="palette" size={19} />
    </button>
    {#if app.settings.mesh.enabled}
      <button class="tool" class:on={app.mesh.show} onclick={toggleSurface} use:tip={t("view.surface")} aria-label={t("view.surface")}>
        <Icon name="surface" size={19} />
      </button>
    {/if}
  </div>

  <div class="group glass">
    <button class="tool" onclick={() => (app.exportOpen = true)} disabled={!app.selection.length} use:tip={t("view.export")} aria-label={t("view.export")}>
      <Icon name="share" size={19} />
    </button>
  </div>

  {#if popover}
    <div class="pop glass-strong" bind:this={popEl} transition:scale={{ start: 0.95, duration: 160 }}>
      <div class="label">{t("view.colors")}</div>
      <div class="colors">
        {#each COLORS as c (c.value)}
          <button class="color" class:on={app.settings.view.colorMode === c.value} onclick={() => updateSettings({ view: { colorMode: c.value } })}>
            <span class="sw">
              {#if c.value === "height" || c.value === "distance"}
                <span class="ramp" style="background:{rampCss()}"></span>
              {:else if c.value === "mono"}
                <span class="ramp" style="background:var(--text)"></span>
              {:else}
                <span class="dots">
                  {#each [0, 1, 2, 3] as i}<i style="background:{scanColorCss(i)}"></i>{/each}
                </span>
              {/if}
            </span>
            <span>{t(c.key)}</span>
          </button>
        {/each}
      </div>
      <div class="row">
        <span>{t("view.pointSize")}</span>
      </div>
      <Slider
        value={app.settings.view.pointSize}
        min={0.4}
        max={2.5}
        step={0.05}
        label={t("view.pointSize")}
        oninput={(v) => {
          app.settings.view.pointSize = v;
        }}
        onchange={(v) => updateSettings({ view: { pointSize: v } })}
      />
      <div class="row">
        <span>{t("view.ceiling")}</span>
        <Toggle checked={clipOn} onchange={setClip} label={t("view.ceiling")} />
      </div>
      {#if clipOn}
        <Slider
          value={Math.min(range.hi, Math.max(range.lo, clipZ))}
          min={range.lo}
          max={range.hi}
          step={10}
          label={t("view.ceiling")}
          oninput={(v) => {
            app.clipMode = "on";
            app.clipZ = v;
          }}
        />
      {/if}
    </div>
  {/if}

  {#if app.mesh.show && app.settings.mesh.enabled}
    <MeshCard />
  {/if}
</div>

<style>
  .tools {
    position: absolute;
    top: calc(var(--titlebar) + 2px);
    right: var(--gap);
    z-index: 20;
    display: flex;
    flex-direction: column;
    align-items: flex-end;
    gap: 10px;
  }
  .group {
    display: flex;
    flex-direction: column;
    gap: 2px;
    padding: 4px;
    border-radius: 16px;
  }
  .tool {
    width: 40px;
    height: 40px;
    border-radius: 12px;
    display: grid;
    place-items: center;
    color: var(--text-2);
    transition: background 0.18s, color 0.18s, transform 0.2s var(--spring);
  }
  .tool:hover:not(:disabled) {
    background: var(--fill);
    color: var(--text);
  }
  .tool:active:not(:disabled) {
    transform: scale(0.92);
  }
  .tool.on {
    background: var(--accent);
    color: #fff;
    box-shadow: 0 2px 10px color-mix(in srgb, var(--accent) 45%, transparent);
  }
  .tool:disabled {
    opacity: 0.35;
  }
  .pop {
    position: absolute;
    right: 58px;
    top: 58px;
    width: 250px;
    padding: 12px;
    border-radius: 16px;
    transform-origin: top right;
  }
  .label {
    font-size: 11px;
    font-weight: 650;
    color: var(--text-2);
    text-transform: uppercase;
    letter-spacing: 0.04em;
    margin-bottom: 6px;
  }
  .colors {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 6px;
    margin-bottom: 12px;
  }
  .color {
    display: flex;
    flex-direction: column;
    align-items: flex-start;
    gap: 6px;
    padding: 8px;
    border-radius: 10px;
    background: var(--fill);
    text-align: left;
    font-weight: 560;
    box-shadow: inset 0 0 0 1.5px transparent;
    transition: box-shadow 0.18s, background 0.18s;
  }
  .color:hover {
    background: var(--fill-2);
  }
  .color.on {
    box-shadow: inset 0 0 0 2px var(--accent);
    background: var(--accent-soft);
  }
  .sw {
    width: 100%;
    height: 8px;
  }
  .ramp {
    display: block;
    height: 8px;
    border-radius: 999px;
    opacity: 0.95;
  }
  .dots {
    display: flex;
    gap: 4px;
  }
  .dots i {
    width: 8px;
    height: 8px;
    border-radius: 50%;
  }
  .row {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin: 8px 0 2px;
    font-weight: 560;
  }
</style>

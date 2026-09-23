<script lang="ts">
  import { fly } from "svelte/transition";
  import { B } from "../api";
  import { compact, fixed } from "../format";
  import { t } from "../i18n.svelte";
  import { app, buildMesh, refreshMeshKey, updateSettings } from "../state.svelte";
  import type { Quality } from "../types";
  import Icon from "./Icon.svelte";
  import Button from "./ui/Button.svelte";
  import Segmented from "./ui/Segmented.svelte";

  let current = $derived(app.mesh.key !== null && app.mesh.key === app.mesh.wantKey);
  let outdated = $derived(app.mesh.key !== null && !current && app.mesh.wantKey !== null);

  $effect(() => {
    // Selection or poses changed: ask what the mesh for this view would be.
    void app.selection.join();
    void app.library;
    void app.geometryVersion;
    void refreshMeshKey();
  });
</script>

<div class="card glass-strong" transition:fly={{ x: 16, duration: 240 }}>
  <div class="head">
    <Icon name="surface" size={17} />
    <span>{t("view.surface")}</span>
  </div>

  {#if app.mesh.busy}
    <div class="busy">
      <span class="ring spin"></span>
      <div>
        <div class="strong">{t("mesh.building")}</div>
        <div class="muted small">{app.mesh.stage ? t(`mesh.${app.mesh.stage}`) : ""}</div>
      </div>
    </div>
    <Button size="sm" variant="ghost" onclick={() => B().meshCancel()}>{t("common.cancel")}</Button>
  {:else if current && app.mesh.info}
    <div class="muted small tnum">
      {t("mesh.info", { tris: compact(app.mesh.info.tris), cell: fixed(app.mesh.info.cellMm, 1) })}
    </div>
    <Button size="sm" variant="secondary" icon="refresh" onclick={buildMesh}>{t("mesh.rebuild")}</Button>
  {:else}
    <div class="muted small">{outdated ? t("mesh.outdated") : t("mesh.none")}</div>
    <Segmented
      size="sm"
      stretch
      value={app.settings.mesh.quality}
      options={[
        { value: "medium" as Quality, label: t("settings.qMedium") },
        { value: "high" as Quality, label: t("settings.qHigh") },
        { value: "max" as Quality, label: t("settings.qMax") },
      ]}
      onchange={(q) => updateSettings({ mesh: { quality: q } })}
    />
    <Button size="md" variant="primary" icon="sparkle" wide disabled={!app.selection.length || app.scanning} onclick={buildMesh}>
      {outdated ? t("mesh.rebuild") : t("mesh.build")}
    </Button>
  {/if}
</div>

<style>
  .card {
    width: 250px;
    padding: 12px;
    border-radius: 16px;
    display: flex;
    flex-direction: column;
    gap: 10px;
  }
  .head {
    display: flex;
    align-items: center;
    gap: 8px;
    font-weight: 650;
  }
  .busy {
    display: flex;
    align-items: center;
    gap: 10px;
  }
  .ring {
    width: 22px;
    height: 22px;
    border-radius: 50%;
    border: 2.5px solid var(--fill-3);
    border-top-color: var(--accent);
    flex: none;
  }
  .strong {
    font-weight: 600;
  }
  .small {
    font-size: 12px;
  }
</style>

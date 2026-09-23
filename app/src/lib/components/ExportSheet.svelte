<script lang="ts">
  import { fade, fly } from "svelte/transition";
  import { B } from "../api";
  import { compact } from "../format";
  import { t } from "../i18n.svelte";
  import { app, describeSelection, toast } from "../state.svelte";
  import Icon from "./Icon.svelte";
  import Button from "./ui/Button.svelte";

  let busy = $state<string | null>(null);

  let scans = $derived(app.visibleScans);
  let points = $derived(scans.reduce((a, s) => a + s.points, 0));
  let single = $derived(scans.length === 1 ? scans[0] : null);
  let rawOk = $derived(!!single && single.kind !== "ply");
  let meshOk = $derived(app.settings.mesh.enabled && app.mesh.key !== null && app.mesh.key === app.mesh.wantKey);

  function baseName(): string {
    const name = describeSelection().replace(/[\\/:*?"<>|·]+/g, " ").replace(/\s+/g, " ").trim();
    return name || "scan";
  }

  function close() {
    if (!busy) app.exportOpen = false;
  }

  async function run(kind: string, ext: string, fn: (path: string) => Promise<unknown>) {
    const path = await B().saveAs(`${baseName()}.${ext}`, [{ name: ext.toUpperCase(), extensions: [ext] }]);
    if (!path) return;
    busy = kind;
    try {
      await fn(path);
      const name = path.split(/[\\/]/).pop() ?? path;
      toast(t("export.saved", { name }), "success", { label: t("export.show"), run: () => B().reveal(path) });
      app.exportOpen = false;
    } catch (e) {
      toast(`${t("export.failed")}: ${e}`, "error");
    } finally {
      busy = null;
    }
  }
</script>

<svelte:window onkeydown={(e) => e.key === "Escape" && close()} />

<div class="scrim" transition:fade={{ duration: 200 }} onclick={close} role="presentation" data-modal-open></div>
<div class="sheet glass-strong" role="dialog" aria-modal="true" transition:fly={{ y: 24, duration: 300 }}>
  <header>
    <div>
      <h2>{t("export.title")}</h2>
      <div class="muted tnum">{describeSelection()} · {compact(points)}</div>
    </div>
    <button class="close" onclick={close} aria-label={t("common.close")}><Icon name="x" size={16} stroke={2} /></button>
  </header>

  <div class="opts">
    <div class="opt">
      <div class="oi c1"><Icon name="points" size={20} /></div>
      <div class="txt">
        <div class="title">{t("export.cloud")}</div>
        <div class="muted small">{t("export.cloudDesc")}</div>
      </div>
      <Button variant="tinted" disabled={!scans.length || !!busy} onclick={() => run("cloud", "ply", (p) => B().exportCloud([...app.selection], p, app.settings.view.colorMode))}>
        {busy === "cloud" ? "…" : t("export.save")}
      </Button>
    </div>

    <div class="opt" class:off={!rawOk}>
      <div class="oi c2"><Icon name="file" size={20} /></div>
      <div class="txt">
        <div class="title">{t("export.raw")}</div>
        <div class="muted small">{rawOk ? t("export.rawDesc") : t("export.rawOnlySingle")}</div>
      </div>
      <Button variant="tinted" disabled={!rawOk || !!busy} onclick={() => run("raw", "bin", (p) => B().exportRaw(single!.id, p))}>
        {busy === "raw" ? "…" : t("export.save")}
      </Button>
    </div>

    {#if app.settings.mesh.enabled}
      <div class="opt" class:off={!meshOk}>
        <div class="oi c3"><Icon name="surface" size={20} /></div>
        <div class="txt">
          <div class="title">{t("export.mesh")}</div>
          <div class="muted small">{meshOk ? t("export.meshDesc") : t("export.meshNone")}</div>
        </div>
        <div class="fmt">
          {#each ["ply", "obj", "stl"] as ext}
            <Button size="sm" variant="tinted" disabled={!meshOk || !!busy} onclick={() => run(`mesh-${ext}`, ext, (p) => B().exportMesh(p))}>
              {busy === `mesh-${ext}` ? "…" : ext.toUpperCase()}
            </Button>
          {/each}
        </div>
      </div>
    {/if}
  </div>
  <div class="foot faint small">{t("export.units")}</div>
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
    width: min(540px, calc(100vw - 48px));
    border-radius: 22px;
    padding: 18px 18px 14px;
  }
  header {
    display: flex;
    align-items: flex-start;
    justify-content: space-between;
    margin: 2px 2px 14px 6px;
  }
  h2 {
    margin: 0 0 2px;
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
  .opts {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }
  .opt {
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 12px;
    border-radius: 14px;
    background: var(--card);
    box-shadow: inset 0 0 0 0.5px var(--glass-border);
    transition: opacity 0.2s;
  }
  .opt.off {
    opacity: 0.6;
  }
  .oi {
    width: 40px;
    height: 40px;
    flex: none;
    border-radius: 11px;
    display: grid;
    place-items: center;
    color: #fff;
  }
  .c1 {
    background: linear-gradient(135deg, #34c3ff, #0a84ff);
  }
  .c2 {
    background: linear-gradient(135deg, #a0a0a8, #6e6e78);
  }
  .c3 {
    background: linear-gradient(135deg, #bf5af2, #5e5ce6);
  }
  .txt {
    flex: 1;
    min-width: 0;
  }
  .title {
    font-weight: 650;
  }
  .small {
    font-size: 12px;
  }
  .fmt {
    display: flex;
    gap: 4px;
  }
  .foot {
    text-align: center;
    margin-top: 12px;
  }
</style>

<script lang="ts">
  // Everything that floats over the viewport for a moment: job progress,
  // the alignment review, toasts, confirmations and the drop target.
  import { fade, fly, scale } from "svelte/transition";
  import { fixed } from "../format";
  import { t } from "../i18n.svelte";
  import { app, cancelMerge, decideMerge } from "../state.svelte";
  import Icon from "./Icon.svelte";
  import Button from "./ui/Button.svelte";

  let pending = $derived(app.merge?.pending ?? null);
  let calibPill = $derived(app.calib.running && !app.settingsOpen);
</script>

<!-- Job progress -->
{#if (app.merge && !pending) || calibPill}
  <div class="pill glass" transition:fly={{ y: -10, duration: 250 }}>
    <span class="ring spin"></span>
    {#if app.merge && !pending}
      <div class="txt">
        <strong>{t("merge.running")}</strong>
        <span class="muted">
          {#if app.merge.total > 1}{t("merge.step", { i: app.merge.index, n: app.merge.total })} · {/if}{app.merge.text}
        </span>
      </div>
      <button class="x" onclick={cancelMerge} aria-label={t("common.cancel")}><Icon name="x" size={14} stroke={2.2} /></button>
    {:else}
      <div class="txt">
        <strong>{t("settings.calibrating")}</strong>
        <span class="muted">{app.calib.stage ? t(`settings.calibStage.${app.calib.stage}`) : ""} · {Math.round(app.calib.fraction * 100)}%</span>
      </div>
      <button class="x" onclick={() => { app.settingsSection = "calibration"; app.settingsOpen = true; }} aria-label="open">
        <Icon name="chevronRight" size={14} stroke={2.2} />
      </button>
    {/if}
  </div>
{/if}

<!-- Alignment review -->
{#if pending}
  <div class="review glass-strong" transition:fly={{ y: 16, duration: 300 }}>
    <div class="icon" class:poor={pending.verdict === "poor"}>
      <Icon name="alert" size={20} />
    </div>
    <div class="body">
      <div class="title">{t("merge.reviewTitle")}</div>
      <div class="verdict">{pending.verdict === "poor" ? t("merge.poor") : t("merge.check")}:
        {pending.reasons.map((r) => t(`merge.reason.${r}`)).join(", ")}.</div>
      <div class="muted small tnum">
        {t("merge.stats", { overlap: Math.round(pending.overlap * 100), rmse: fixed(pending.rmse, 0) })} · {t("merge.reviewBody")}
      </div>
    </div>
    <div class="actions">
      <Button variant="secondary" onclick={() => decideMerge(false)}>{t("merge.reject")}</Button>
      <Button variant="primary" onclick={() => decideMerge(true)}>{t("merge.accept")}</Button>
    </div>
  </div>
{/if}

<!-- Toasts -->
<div class="toasts">
  {#each app.toasts as tt (tt.id)}
    <div class="toast glass {tt.kind}" in:fly={{ y: -12, duration: 260 }} out:fade={{ duration: 180 }}>
      <span class="ti">
        <Icon name={tt.kind === "error" ? "alert" : tt.kind === "success" ? "check" : "info"} size={15} stroke={2.2} />
      </span>
      <span>{tt.text}</span>
      {#if tt.action}
        <button class="act" onclick={() => { tt.action?.run(); app.toasts = app.toasts.filter((x) => x.id !== tt.id); }}>{tt.action.label}</button>
      {/if}
    </div>
  {/each}
</div>

<!-- Confirmation -->
{#if app.confirm}
  {@const c = app.confirm}
  <div class="scrim" transition:fade={{ duration: 180 }} data-modal-open></div>
  <div class="dialog-wrap" role="presentation" onkeydown={(e) => e.key === "Escape" && c.resolve(false)}>
    <div class="dialog glass-strong" role="alertdialog" aria-modal="true" transition:scale={{ start: 0.94, duration: 220 }}>
      <h3>{c.title}</h3>
      <p>{c.body}</p>
      <div class="dactions">
        <Button variant="secondary" wide onclick={() => c.resolve(false)}>{t("common.cancel")}</Button>
        <Button variant={c.danger ? "danger" : "primary"} wide onclick={() => c.resolve(true)}>{c.ok}</Button>
      </div>
    </div>
  </div>
{/if}

<!-- Drop target -->
{#if app.dropHover}
  <div class="drop" transition:fade={{ duration: 150 }}>
    <div class="drop-inner">
      <Icon name="download" size={34} />
      <span>{t("library.dropHint")}</span>
    </div>
  </div>
{/if}

<style>
  .pill {
    position: absolute;
    top: calc(var(--titlebar) + 8px);
    left: 50%;
    translate: -50% 0;
    z-index: 30;
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 8px 8px 8px 14px;
    border-radius: 999px;
    max-width: 60vw;
  }
  .txt {
    display: flex;
    gap: 8px;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .ring {
    width: 16px;
    height: 16px;
    flex: none;
    border-radius: 50%;
    border: 2px solid var(--fill-3);
    border-top-color: var(--accent);
  }
  .x {
    width: 24px;
    height: 24px;
    border-radius: 50%;
    display: grid;
    place-items: center;
    background: var(--fill);
    color: var(--text-2);
  }
  .x:hover {
    background: var(--fill-2);
    color: var(--text);
  }
  .review {
    position: absolute;
    left: 50%;
    bottom: 150px;
    translate: -50% 0;
    z-index: 30;
    display: flex;
    align-items: center;
    gap: 14px;
    width: min(640px, 70vw);
    padding: 14px 14px 14px 16px;
    border-radius: 20px;
  }
  .icon {
    width: 38px;
    height: 38px;
    flex: none;
    border-radius: 12px;
    display: grid;
    place-items: center;
    color: var(--orange);
    background: color-mix(in srgb, var(--orange) 16%, transparent);
  }
  .icon.poor {
    color: var(--red);
    background: color-mix(in srgb, var(--red) 16%, transparent);
  }
  .body {
    flex: 1;
    min-width: 0;
  }
  .title {
    font-weight: 650;
    font-size: 14px;
  }
  .verdict {
    margin: 2px 0;
  }
  .small {
    font-size: 12px;
  }
  .actions {
    display: flex;
    gap: 6px;
  }
  .toasts {
    position: absolute;
    top: calc(var(--titlebar) + 8px);
    left: 50%;
    translate: -50% 0;
    z-index: 60;
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
    pointer-events: none;
  }
  .toast {
    display: flex;
    align-items: center;
    gap: 9px;
    padding: 8px 10px 8px 10px;
    border-radius: 999px;
    font-weight: 560;
    pointer-events: auto;
    white-space: nowrap;
  }
  .ti {
    width: 22px;
    height: 22px;
    border-radius: 50%;
    display: grid;
    place-items: center;
    background: var(--fill-2);
  }
  .success .ti {
    background: var(--green);
    color: #fff;
  }
  .error .ti {
    background: var(--red);
    color: #fff;
  }
  .act {
    margin-left: 4px;
    padding: 3px 10px;
    border-radius: 999px;
    background: var(--accent-soft);
    color: var(--accent);
    font-weight: 600;
  }
  .scrim {
    position: fixed;
    inset: 0;
    background: var(--scrim);
    z-index: 90;
    backdrop-filter: blur(2px);
  }
  .dialog-wrap {
    position: fixed;
    inset: 0;
    z-index: 91;
    display: grid;
    place-items: center;
  }
  .dialog {
    width: 340px;
    padding: 22px 20px 16px;
    border-radius: 18px;
    text-align: center;
  }
  h3 {
    margin: 0 0 6px;
    font-size: 16px;
    font-weight: 700;
  }
  p {
    margin: 0 0 18px;
    color: var(--text-2);
  }
  .dactions {
    display: flex;
    gap: 8px;
  }
  .drop {
    position: fixed;
    inset: 0;
    z-index: 95;
    padding: 18px;
    background: color-mix(in srgb, var(--accent) 10%, transparent);
    backdrop-filter: blur(4px);
  }
  .drop-inner {
    height: 100%;
    border: 2.5px dashed var(--accent);
    border-radius: 24px;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    gap: 12px;
    color: var(--accent);
    font-size: 16px;
    font-weight: 650;
  }
</style>

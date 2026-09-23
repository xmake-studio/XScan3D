<script lang="ts">
  import { compact, duration } from "../format";
  import { plural, t } from "../i18n.svelte";
  import { app, itemTitle } from "../state.svelte";
  import type { LibraryItem } from "../types";
  import { loadThumb } from "../viewer";
  import Icon from "./Icon.svelte";

  let {
    item,
    selected,
    multi,
    onpick,
    onmenu,
    onrename,
  }: {
    item: LibraryItem;
    selected: boolean;
    multi: boolean;
    onpick: (e: MouseEvent) => void;
    onmenu: (e: MouseEvent) => void;
    onrename: (name: string) => void;
  } = $props();

  let editing = $state(false);
  let draft = $state("");
  let input: HTMLInputElement | undefined = $state();

  const thumbId = $derived(item.id);
  $effect(() => {
    void loadThumb(thumbId);
    if (item.kind === "group") for (const s of item.scans) void loadThumb(s.id);
  });
  let thumb = $derived(app.thumbs[item.id] ?? (item.kind === "group" ? item.scans.map((s) => app.thumbs[s.id]).find(Boolean) : undefined));

  let points = $derived(item.scans.reduce((a, s) => a + s.points, 0));
  let meta = $derived.by(() => {
    if (item.kind === "group") return `${item.scans.length} ${plural("scans", item.scans.length)} · ${t("library.pointsShort", { n: compact(points) })}`;
    const s = item.scans[0];
    const parts = [t("library.pointsShort", { n: compact(points) })];
    if (s.duration > 0) parts.push(duration(s.duration));
    return parts.join(" · ");
  });
  let loading = $derived(item.scans.some((s) => app.loading.includes(s.id)));
  let interrupted = $derived(item.kind === "scan" && !item.scans[0].complete);
  let imported = $derived(item.kind === "scan" && item.scans[0].kind !== "device");

  function beginRename() {
    draft = item.name ?? itemTitle(item);
    editing = true;
    queueMicrotask(() => input?.select());
  }
  function finish(save: boolean) {
    if (!editing) return;
    editing = false;
    if (save && draft.trim() !== (item.name ?? itemTitle(item))) onrename(draft);
  }

  export function rename() {
    beginRename();
  }
</script>

<div
  class="row"
  class:selected
  role="option"
  aria-selected={selected}
  tabindex="0"
  onclick={onpick}
  oncontextmenu={(e) => {
    e.preventDefault();
    onmenu(e);
  }}
  onkeydown={(e) => {
    if (e.key === "F2") beginRename();
  }}
>
  <span class="check" class:shown={multi} class:on={selected} aria-hidden="true">
    {#if selected}<Icon name="check" size={11} stroke={3} />{/if}
  </span>
  <div class="thumb">
    {#if thumb}
      <img src={thumb} alt="" draggable="false" />
    {:else}
      <Icon name={item.kind === "group" ? "layers" : "points"} size={20} />
    {/if}
    {#if item.kind === "group"}<span class="badge"><Icon name="layers" size={11} stroke={2.2} /></span>{/if}
  </div>
  <div class="text">
    {#if editing}
      <input
        bind:this={input}
        bind:value={draft}
        onclick={(e) => e.stopPropagation()}
        onblur={() => finish(true)}
        onkeydown={(e) => {
          if (e.key === "Enter") finish(true);
          else if (e.key === "Escape") finish(false);
          e.stopPropagation();
        }}
      />
    {:else}
      <div class="name" ondblclick={(e) => { e.stopPropagation(); beginRename(); }} role="presentation">{itemTitle(item)}</div>
    {/if}
    <div class="meta">
      {#if interrupted}<span class="tag warn">{t("library.interrupted")}</span>{/if}
      {#if imported}<span class="tag">{t("library.imported")}</span>{/if}
      <span class="tnum">{meta}</span>
    </div>
  </div>
  {#if loading}
    <span class="spinner spin"></span>
  {:else}
    <button class="more" aria-label="…" onclick={(e) => { e.stopPropagation(); onmenu(e); }}>
      <Icon name="more" size={16} />
    </button>
  {/if}
</div>

<style>
  .row {
    position: relative;
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 7px 8px 7px 8px;
    border-radius: 12px;
    transition: background 0.18s;
    cursor: default;
  }
  .row:hover {
    background: var(--fill);
  }
  .row.selected {
    background: var(--accent-soft);
  }
  .row:focus-visible {
    outline: 2px solid var(--accent);
    outline-offset: -2px;
  }
  .check {
    position: absolute;
    left: 3px;
    top: 3px;
    z-index: 2;
    width: 18px;
    height: 18px;
    border-radius: 50%;
    display: grid;
    place-items: center;
    background: var(--glass-strong);
    box-shadow: 0 0 0 1.5px var(--fill-3), var(--shadow-sm);
    color: #fff;
    opacity: 0;
    transform: scale(0.6);
    transition: opacity 0.18s, transform 0.25s var(--spring), background 0.15s;
  }
  .row:hover .check,
  .check.shown {
    opacity: 1;
    transform: scale(1);
  }
  .check.on {
    background: var(--accent);
    box-shadow: 0 0 0 1.5px var(--accent), var(--shadow-sm);
  }
  .thumb {
    position: relative;
    width: 64px;
    height: 44px;
    flex: none;
    border-radius: 9px;
    overflow: hidden;
    display: grid;
    place-items: center;
    color: var(--text-3);
    background: linear-gradient(135deg, var(--fill-2), var(--fill));
    box-shadow: inset 0 0 0 0.5px var(--glass-border);
  }
  .thumb img {
    width: 100%;
    height: 100%;
    object-fit: cover;
    display: block;
    animation: fadein 0.4s var(--ease);
  }
  @keyframes fadein {
    from {
      opacity: 0;
    }
  }
  .badge {
    position: absolute;
    right: 3px;
    bottom: 3px;
    padding: 2px;
    border-radius: 5px;
    background: rgba(0, 0, 0, 0.5);
    color: #fff;
    backdrop-filter: blur(6px);
  }
  .text {
    flex: 1;
    min-width: 0;
  }
  .name {
    font-weight: 600;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  input {
    width: 100%;
    font-weight: 600;
    border: 0;
    border-radius: 5px;
    padding: 1px 4px;
    margin: -1px -4px;
    background: var(--field);
    box-shadow: 0 0 0 2px var(--accent);
  }
  .meta {
    display: flex;
    align-items: center;
    gap: 5px;
    margin-top: 2px;
    color: var(--text-2);
    font-size: 12px;
    white-space: nowrap;
    overflow: hidden;
  }
  .tag {
    padding: 0 6px;
    border-radius: 5px;
    background: var(--fill-2);
    font-size: 10.5px;
    font-weight: 600;
    color: var(--text-2);
  }
  .tag.warn {
    background: color-mix(in srgb, var(--orange) 18%, transparent);
    color: var(--orange);
  }
  .more {
    width: 26px;
    height: 26px;
    flex: none;
    border-radius: 7px;
    display: grid;
    place-items: center;
    color: var(--text-2);
    opacity: 0;
    transition: opacity 0.15s, background 0.15s;
  }
  .row:hover .more,
  .row.selected .more {
    opacity: 1;
  }
  .more:hover {
    background: var(--fill-2);
    color: var(--text);
  }
  .spinner {
    width: 16px;
    height: 16px;
    margin: 0 5px;
    border-radius: 50%;
    border: 2px solid var(--fill-3);
    border-top-color: var(--accent);
  }
</style>

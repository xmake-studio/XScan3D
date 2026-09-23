<script lang="ts">
  import { flip } from "svelte/animate";
  import { fly, slide } from "svelte/transition";
  import { B } from "../api";
  import { compact } from "../format";
  import { t } from "../i18n.svelte";
  import {
    app, deleteItems, mergeSelection, openFiles, renameItem, select, startCalibration, ungroupItems,
  } from "../state.svelte";
  import { tip } from "../tooltip";
  import type { LibraryItem } from "../types";
  import Icon from "./Icon.svelte";
  import LibraryRow from "./LibraryRow.svelte";
  import Button from "./ui/Button.svelte";
  import Menu, { type MenuItem } from "./ui/Menu.svelte";

  let menu = $state<{ x: number; y: number; items: MenuItem[] } | null>(null);
  // Plain, not reactive: these handles are only called imperatively (rename),
  // and writing them during render would re-run the list for nothing.
  const rows: Record<string, LibraryRow> = {};

  let multi = $derived(app.selection.length > 1);

  function pick(it: LibraryItem, e: MouseEvent) {
    if (e.shiftKey) select(it.id, "range");
    else if (e.ctrlKey || e.metaKey || (e.target as HTMLElement).closest(".check")) select(it.id, "toggle");
    else select(it.id, "single");
  }

  function openMenu(it: LibraryItem, e: MouseEvent) {
    if (!app.selection.includes(it.id)) select(it.id, "single");
    const ids = app.selection.includes(it.id) ? [...app.selection] : [it.id];
    const single = ids.length === 1;
    const raw = it.kind === "scan" && it.scans[0].kind !== "ply";
    const items: MenuItem[] = [];
    if (single) items.push({ label: t("library.rename"), icon: "pencil", run: () => rows[it.id]?.rename() });
    items.push({ label: t("library.export"), icon: "share", run: () => (app.exportOpen = true) });
    if (single && it.kind === "scan")
      items.push({
        label: t("library.reveal"),
        icon: "folder",
        run: () => B().reveal(`${app.library.root}\\${it.scans[0].file.replaceAll("/", "\\")}`),
      });
    if (!single) items.push({ label: t("library.merge"), icon: "link", run: () => mergeSelection() });
    if (single && raw)
      items.push({
        label: t("library.calibrate"),
        icon: "target",
        run: () => {
          app.settingsSection = "calibration";
          app.settingsOpen = true;
          void startCalibration();
        },
      });
    if (app.items.some((x) => ids.includes(x.id) && x.kind === "group"))
      items.push({ label: t("library.ungroup"), icon: "unlink", run: () => ungroupItems(ids) });
    items.push({ label: t("library.remove"), icon: "trash", danger: true, separator: true, run: () => deleteItems(ids) });
    menu = { x: e.clientX, y: e.clientY, items };
  }

  function keydown(e: KeyboardEvent) {
    if ((e.target as HTMLElement).tagName === "INPUT") return;
    if (e.key === "Delete" && app.selection.length) {
      e.preventDefault();
      void deleteItems([...app.selection]);
    } else if ((e.key === "a" || e.key === "A" || e.code === "KeyA") && (e.ctrlKey || e.metaKey)) {
      e.preventDefault();
      app.selection = app.items.map((i) => i.id);
    } else if (e.key === "ArrowDown" || e.key === "ArrowUp") {
      e.preventDefault();
      const ids = app.items.map((i) => i.id);
      const cur = ids.indexOf(app.selection.at(-1) ?? "");
      const next = Math.max(0, Math.min(ids.length - 1, cur + (e.key === "ArrowDown" ? 1 : -1)));
      if (ids[next]) select(ids[next], e.shiftKey ? "range" : "single");
    }
  }
</script>

<aside class="side glass" transition:fly={{ x: -24, duration: 300 }}>
  <header>
    <div class="h">
      <h2>{t("library.title")}</h2>
      {#if app.items.length}<span class="count tnum">{app.items.length}</span>{/if}
    </div>
    <button class="add" onclick={openFiles} use:tip={t("library.open")} aria-label={t("library.open")}>
      <Icon name="plus" size={17} />
    </button>
  </header>

  <div class="list" role="listbox" aria-multiselectable="true" tabindex="-1" onkeydown={keydown}>
    {#if app.scan}
      <div class="live" transition:slide={{ duration: 250 }}>
        <span class="rec"></span>
        <div class="text">
          <div class="name">{t("scan.scanning")}</div>
          <div class="bar"><span style="width:{Math.round(app.scan.progress * 100)}%"></span></div>
        </div>
        <span class="tnum muted">{compact(app.scan.points)}</span>
      </div>
    {/if}
    {#each app.items as it (it.id)}
      <div animate:flip={{ duration: 280 }} in:fly={{ y: -8, duration: 280 }}>
        <LibraryRow
          bind:this={rows[it.id]}
          item={it}
          selected={app.selection.includes(it.id)}
          {multi}
          onpick={(e) => pick(it, e)}
          onmenu={(e) => openMenu(it, e)}
          onrename={(name) => renameItem(it.id, name)}
        />
      </div>
    {:else}
      {#if !app.scan}
        <div class="empty">
          <Icon name="points" size={28} />
          <p>{t("library.empty")}</p>
          <p class="faint">{t("library.emptyHint")}</p>
        </div>
      {/if}
    {/each}
  </div>

  {#if multi}
    <footer transition:slide={{ duration: 220 }}>
      <span class="muted tnum">{t("library.selected", { n: app.selection.length })}</span>
      <div class="actions">
        <Button variant="ghost" size="sm" icon="trash" title={t("library.remove")} onclick={() => deleteItems([...app.selection])} />
        <Button variant="ghost" size="sm" icon="x" title={t("library.clear")} onclick={() => (app.selection = app.selection.slice(-1))} />
        <Button variant="primary" size="sm" icon="link" disabled={!!app.merge} onclick={mergeSelection}>{t("library.merge")}</Button>
      </div>
    </footer>
  {/if}
</aside>

{#if menu}
  <Menu x={menu.x} y={menu.y} items={menu.items} onclose={() => (menu = null)} />
{/if}

<style>
  .side {
    position: absolute;
    top: calc(var(--titlebar) + 2px);
    left: var(--gap);
    bottom: var(--gap);
    width: 300px;
    border-radius: var(--r-xl);
    display: flex;
    flex-direction: column;
    overflow: hidden;
    z-index: 20;
  }
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 14px 12px 8px 16px;
  }
  .h {
    display: flex;
    align-items: baseline;
    gap: 8px;
  }
  h2 {
    margin: 0;
    font-size: 17px;
    font-weight: 700;
    letter-spacing: -0.02em;
  }
  .count {
    color: var(--text-3);
    font-weight: 600;
  }
  .add {
    width: 30px;
    height: 30px;
    border-radius: 9px;
    display: grid;
    place-items: center;
    color: var(--accent);
    background: var(--accent-soft);
    transition: transform 0.2s var(--spring), filter 0.2s;
  }
  .add:hover {
    filter: brightness(1.12);
  }
  .add:active {
    transform: scale(0.92);
  }
  .list {
    flex: 1;
    overflow-y: auto;
    overflow-x: hidden;
    padding: 2px 8px 10px;
    display: flex;
    flex-direction: column;
    gap: 2px;
  }
  .empty {
    margin: 40px 16px;
    text-align: center;
    color: var(--text-2);
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 6px;
  }
  .empty p {
    margin: 0;
  }
  .live {
    display: flex;
    align-items: center;
    gap: 10px;
    padding: 10px 10px;
    margin-bottom: 4px;
    border-radius: 12px;
    background: color-mix(in srgb, var(--red) 9%, transparent);
  }
  .rec {
    width: 10px;
    height: 10px;
    border-radius: 50%;
    background: var(--red);
    box-shadow: 0 0 10px var(--red);
    animation: pulse-soft 1.2s ease-in-out infinite;
    flex: none;
  }
  .live .text {
    flex: 1;
  }
  .live .name {
    font-weight: 600;
  }
  .bar {
    height: 4px;
    margin-top: 5px;
    border-radius: 999px;
    background: var(--fill-2);
    overflow: hidden;
  }
  .bar span {
    display: block;
    height: 100%;
    border-radius: 999px;
    background: var(--red);
    transition: width 0.3s var(--ease);
  }
  footer {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 8px;
    padding: 10px 10px 10px 16px;
    border-top: 1px solid var(--sep);
  }
  .actions {
    display: flex;
    align-items: center;
    gap: 4px;
  }
</style>

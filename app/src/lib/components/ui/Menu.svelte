<script lang="ts" module>
  export interface MenuItem {
    label: string;
    icon?: string;
    danger?: boolean;
    disabled?: boolean;
    run: () => void;
    separator?: boolean;
  }
</script>

<script lang="ts">
  // A context menu at a screen position. Closes on outside click, Escape,
  // or after an item runs.
  import { scale } from "svelte/transition";
  import Icon from "../Icon.svelte";

  let { x, y, items, onclose }: { x: number; y: number; items: MenuItem[]; onclose: () => void } = $props();

  let el: HTMLDivElement | undefined = $state();
  let pos = $state({ left: 0, top: 0 });

  $effect(() => {
    if (!el) return;
    const w = el.offsetWidth, h = el.offsetHeight;
    pos = { left: Math.min(x, window.innerWidth - w - 8), top: Math.min(y, window.innerHeight - h - 8) };
    const away = (e: PointerEvent) => {
      if (el && !el.contains(e.target as Node)) onclose();
    };
    const key = (e: KeyboardEvent) => e.key === "Escape" && onclose();
    const t = setTimeout(() => window.addEventListener("pointerdown", away, true), 0);
    window.addEventListener("keydown", key);
    window.addEventListener("blur", onclose);
    return () => {
      clearTimeout(t);
      window.removeEventListener("pointerdown", away, true);
      window.removeEventListener("keydown", key);
      window.removeEventListener("blur", onclose);
    };
  });
</script>

<div class="menu glass-strong" bind:this={el} style="left:{pos.left}px; top:{pos.top}px" transition:scale={{ start: 0.96, duration: 140 }} role="menu">
  {#each items as it}
    {#if it.separator}<div class="sep"></div>{/if}
    <button
      class="item"
      class:danger={it.danger}
      disabled={it.disabled}
      role="menuitem"
      onclick={() => {
        onclose();
        it.run();
      }}
    >
      {#if it.icon}<Icon name={it.icon} size={15} />{/if}
      <span>{it.label}</span>
    </button>
  {/each}
</div>

<style>
  .menu {
    position: fixed;
    z-index: 200;
    min-width: 210px;
    padding: 5px;
    border-radius: 12px;
    transform-origin: top left;
  }
  .item {
    width: 100%;
    display: flex;
    align-items: center;
    gap: 9px;
    padding: 6px 9px;
    border-radius: 7px;
    text-align: left;
    color: var(--text);
  }
  .item :global(svg) {
    color: var(--text-2);
  }
  .item:hover:not(:disabled) {
    background: var(--accent);
    color: #fff;
  }
  .item:hover:not(:disabled) :global(svg) {
    color: #fff;
  }
  .item:disabled {
    opacity: 0.4;
  }
  .danger {
    color: var(--red);
  }
  .danger :global(svg) {
    color: var(--red);
  }
  .danger:hover:not(:disabled) {
    background: var(--red);
  }
  .sep {
    height: 1px;
    margin: 4px 8px;
    background: var(--sep);
  }
</style>

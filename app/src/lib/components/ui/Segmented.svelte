<script lang="ts" generics="T extends string | number">
  import Icon from "../Icon.svelte";

  interface Option {
    value: T;
    label?: string;
    icon?: string;
    title?: string;
  }
  let {
    options,
    value = $bindable(),
    onchange,
    vertical = false,
    size = "md",
    stretch = false,
  }: { options: Option[]; value: T; onchange?: (v: T) => void; vertical?: boolean; size?: "sm" | "md" | "lg"; stretch?: boolean } = $props();

  let index = $derived(Math.max(0, options.findIndex((o) => o.value === value)));

  function pick(v: T) {
    if (v === value) return;
    value = v;
    onchange?.(v);
  }
</script>

<div class="seg {size}" class:vertical class:stretch style="--n:{options.length}; --i:{index}" role="radiogroup">
  <span class="thumb"></span>
  {#each options as o (o.value)}
    <button class="opt" class:active={o.value === value} role="radio" aria-checked={o.value === value} title={o.title ?? o.label} onclick={() => pick(o.value)}>
      {#if o.icon}<Icon name={o.icon} size={size === "lg" ? 20 : 17} />{/if}
      {#if o.label}<span>{o.label}</span>{/if}
    </button>
  {/each}
</div>

<style>
  .seg {
    position: relative;
    display: inline-grid;
    grid-template-columns: repeat(var(--n), 1fr);
    padding: 2px;
    border-radius: 10px;
    background: var(--fill);
    isolation: isolate;
  }
  .seg.stretch {
    display: grid;
    width: 100%;
  }
  .seg.vertical {
    grid-template-columns: 1fr;
    grid-template-rows: repeat(var(--n), 1fr);
  }
  .thumb {
    position: absolute;
    z-index: -1;
    top: 2px;
    bottom: 2px;
    left: calc(2px + (100% - 4px) / var(--n) * var(--i));
    width: calc((100% - 4px) / var(--n));
    border-radius: 8px;
    background: var(--glass-strong);
    box-shadow: 0 1px 3px rgba(0, 0, 0, 0.18), 0 0 0 0.5px var(--glass-border);
    transition: left 0.32s var(--ease), top 0.32s var(--ease);
  }
  :global([data-theme="light"]) .thumb {
    background: #fff;
  }
  .vertical .thumb {
    left: 2px;
    right: 2px;
    width: auto;
    top: calc(2px + (100% - 4px) / var(--n) * var(--i));
    bottom: auto;
    height: calc((100% - 4px) / var(--n));
  }
  .opt {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 6px;
    padding: 5px 12px;
    border-radius: 8px;
    color: var(--text-2);
    font-weight: 500;
    white-space: nowrap;
    transition: color 0.2s;
  }
  .sm .opt {
    padding: 3px 10px;
    font-size: 12px;
  }
  .lg .opt {
    padding: 8px 9px;
  }
  .opt:hover,
  .opt.active {
    color: var(--text);
  }
</style>

<script lang="ts">
  // A compact numeric field with a unit; arrow keys nudge (Shift x10).
  let {
    value,
    min = -Infinity,
    max = Infinity,
    step = 1,
    digits = 1,
    unit = "",
    width = 92,
    onchange,
    disabled = false,
  }: {
    value: number;
    min?: number;
    max?: number;
    step?: number;
    digits?: number;
    unit?: string;
    width?: number;
    onchange: (v: number) => void;
    disabled?: boolean;
  } = $props();

  let text = $state("");
  let editing = $state(false);

  $effect(() => {
    if (!editing) text = fmt(value);
  });

  function fmt(v: number) {
    return Number.isFinite(v) ? v.toFixed(digits) : "";
  }

  function commit() {
    editing = false;
    const v = parseFloat(text.replace(",", "."));
    if (Number.isFinite(v)) {
      const c = Math.min(max, Math.max(min, v));
      if (c !== value) onchange(c);
      text = fmt(c);
    } else text = fmt(value);
  }

  function nudge(dir: number, big: boolean) {
    const c = Math.min(max, Math.max(min, +(value + dir * step * (big ? 10 : 1)).toFixed(6)));
    onchange(c);
  }

  function keydown(e: KeyboardEvent) {
    const el = e.currentTarget as HTMLInputElement;
    if (e.key === "Enter") el.blur();
    else if (e.key === "Escape") {
      text = fmt(value);
      el.blur();
    } else if (e.key === "ArrowUp" || e.key === "ArrowDown") {
      e.preventDefault();
      nudge(e.key === "ArrowUp" ? 1 : -1, e.shiftKey);
    }
  }
</script>

<label class="nf" class:disabled style="width:{width}px">
  <input
    type="text"
    inputmode="decimal"
    {disabled}
    bind:value={text}
    onfocus={(e) => {
      editing = true;
      (e.currentTarget as HTMLInputElement).select();
    }}
    onblur={commit}
    onkeydown={keydown}
  />
  {#if unit}<span class="unit">{unit}</span>{/if}
</label>

<style>
  .nf {
    display: flex;
    align-items: center;
    gap: 4px;
    height: 28px;
    padding: 0 8px;
    border-radius: 7px;
    background: var(--field);
    box-shadow: inset 0 0 0 0.5px var(--sep);
    transition: box-shadow 0.2s;
    flex: none;
  }
  .nf:focus-within {
    box-shadow: 0 0 0 3px var(--accent-soft), inset 0 0 0 1px var(--accent);
  }
  .nf.disabled {
    opacity: 0.45;
  }
  input {
    flex: 1;
    min-width: 0;
    border: 0;
    background: transparent;
    text-align: right;
    font-variant-numeric: tabular-nums;
    padding: 0;
  }
  .unit {
    color: var(--text-2);
    font-size: 12px;
  }
</style>

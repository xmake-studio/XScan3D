<script lang="ts">
  let {
    checked = $bindable(false),
    disabled = false,
    onchange,
    label = "",
  }: { checked?: boolean; disabled?: boolean; onchange?: (v: boolean) => void; label?: string } = $props();

  function flip() {
    if (disabled) return;
    checked = !checked;
    onchange?.(checked);
  }
</script>

<button class="toggle" class:on={checked} role="switch" aria-checked={checked} aria-label={label} {disabled} onclick={flip}>
  <span class="knob"></span>
</button>

<style>
  .toggle {
    position: relative;
    width: 42px;
    height: 25px;
    border-radius: 999px;
    background: var(--fill-3);
    transition: background 0.25s var(--ease);
    flex: none;
  }
  .toggle.on {
    background: var(--green);
  }
  .toggle:disabled {
    opacity: 0.45;
  }
  .knob {
    position: absolute;
    top: 2px;
    left: 2px;
    width: 21px;
    height: 21px;
    border-radius: 999px;
    background: #fff;
    box-shadow: 0 2px 5px rgba(0, 0, 0, 0.22), 0 0 0 0.5px rgba(0, 0, 0, 0.06);
    transition: transform 0.28s var(--spring), width 0.2s var(--ease);
  }
  .on .knob {
    transform: translateX(17px);
  }
  .toggle:active:not(:disabled) .knob {
    width: 25px;
  }
  .on:active:not(:disabled) .knob {
    transform: translateX(13px);
  }
</style>

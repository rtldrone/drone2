<script>
    import {overrideSensors} from "./stores";
</script>


<div class="text-4xl font-bold text-red-600 fixed">
    Disable Distance Sensors
</div>

<div class="ml-[150px] mt-[70px] fixed">
<input type="checkbox" bind:checked="{$overrideSensors}" class="toggle toggle-primary toggle-lg" />
</div>

